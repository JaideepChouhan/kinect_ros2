#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
ex09_mic_visualizer_node.py  –  Real-time visualisation of the Kinect mic array.

Subscribes to /kinect/audio/raw (4-channel interleaved PCM) and
/kinect/audio/mono (channel-0 beam-formed signal) and renders a composite
diagnostic figure that is published as a ROS 2 Image message.

The figure layout (640×480 pixels, RGB8):

┌─────────────────────────────────────────────────────┐
│  CH 0 ─────── waveform (1 s rolling buffer) ──────  │  ← blue
│  CH 1 ─────── waveform ──────────────────────────  │  ← orange
│  CH 2 ─────── waveform ──────────────────────────  │  ← green
│  CH 3 ─────── waveform ──────────────────────────  │  ← red
├──────────────────────────┬──────────────────────────┤
│  FFT spectrum (CH 0)     │  RMS level meters        │
│  0 – 8000 Hz log plot    │  ████ CH 0               │
│                          │  ██   CH 1               │
│                          │  ███  CH 2               │
│                          │  ███  CH 3               │
└──────────────────────────┴──────────────────────────┘

Published topics
----------------
  /kinect/audio/visualizer  (sensor_msgs/Image, rgb8)
      Rendered diagnostic figure, ~5 Hz

  /kinect/audio/rms         (std_msgs/String)
      JSON: {"ch0": N, "ch1": N, "ch2": N, "ch3": N} per chunk

Subscribed topics
-----------------
  /kinect/audio/raw    (std_msgs/Int16MultiArray)  – all 4 channels
  /kinect/audio/info   (std_msgs/String)           – JSON metadata

Parameters
----------
  window_s       float  1.0   Rolling waveform window (seconds)
  render_rate_hz float  5.0   Visualisation publish rate (Hz)
  figure_width   int    640   Output image width  (pixels)
  figure_height  int    480   Output image height (pixels)
  fft_bins       int    512   FFT bins for spectrum plot

Dependencies
------------
  pip3 install matplotlib numpy
  (cv_bridge is already a kinect_ros2 dependency)
"""

import json
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String
from cv_bridge import CvBridge

# Matplotlib must be initialised before any pyplot import.
# Use the non-interactive 'Agg' backend so it works headlessly on ROS nodes.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# Colours for the 4 microphone channels (matches standard data-visualisation palette)
_CH_COLOURS = ['#4C8EF0', '#F07A2E', '#3EBD6F', '#E35C5C']
_CH_NAMES   = ['CH 0 (Left)', 'CH 1', 'CH 2', 'CH 3 (Right)']


class MicVisualizerNode(Node):
    """Publishes a real-time composite mic-array diagnostic image."""

    def __init__(self) -> None:
        super().__init__('mic_visualizer_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('window_s',       1.0)
        self.declare_parameter('render_rate_hz', 5.0)
        self.declare_parameter('figure_width',   640)
        self.declare_parameter('figure_height',  480)
        self.declare_parameter('fft_bins',       512)

        p = self.get_parameter
        self._window_s    = p('window_s').value
        self._render_rate = p('render_rate_hz').value
        self._fig_w       = p('figure_width').value
        self._fig_h       = p('figure_height').value
        self._fft_bins    = p('fft_bins').value

        # ── Audio metadata (populated from /kinect/audio/info) ─────────────
        self._rate     = 16000          # Hz – updated from info topic
        self._channels = 4              # updated from info topic

        # ── Rolling audio buffers (one deque per channel) ──────────────────
        # Initialised once we know the sample rate
        self._win_samples: int = int(self._window_s * self._rate)
        self._bufs: list[deque] = [
            deque([0] * self._win_samples, maxlen=self._win_samples)
            for _ in range(4)
        ]
        self._rms: list[float] = [0.0, 0.0, 0.0, 0.0]

        # ── cv_bridge & matplotlib figure (pre-allocated) ──────────────────
        self._bridge = CvBridge()
        self._setup_figure()

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_viz = self.create_publisher(Image,  '/kinect/audio/visualizer', 10)
        self._pub_rms = self.create_publisher(String, '/kinect/audio/rms',        10)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Int16MultiArray, '/kinect/audio/raw',  self._raw_cb,  10
        )
        self.create_subscription(
            String, '/kinect/audio/info', self._info_cb, 10
        )

        # ── Render timer ───────────────────────────────────────────────────
        self._render_timer = self.create_timer(
            1.0 / self._render_rate, self._render_and_publish
        )

        self._chunks_received = 0
        self._last_render_t   = time.time()

        self.get_logger().info(
            f'MicVisualizerNode ready  →  /kinect/audio/visualizer '
            f'({self._fig_w}×{self._fig_h}px @ {self._render_rate:.0f} Hz)'
        )

    # ── Figure layout ────────────────────────────────────────────────────────

    def _setup_figure(self) -> None:
        """Create the matplotlib figure and axis objects once."""
        dpi        = 100
        width_in   = self._fig_w  / dpi
        height_in  = self._fig_h  / dpi

        self._fig = plt.figure(
            figsize=(width_in, height_in), dpi=dpi, facecolor='#1A1A2E'
        )

        # Grid: 4 waveform rows on top, spectrum + RMS on the bottom row
        gs = gridspec.GridSpec(
            5, 2,
            figure=self._fig,
            hspace=0.55, wspace=0.35,
            left=0.08, right=0.97,
            top=0.93, bottom=0.07,
        )

        # Waveform axes (rows 0–3, spanning both columns)
        self._ax_wave = []
        for i in range(4):
            ax = self._fig.add_subplot(gs[i, :])
            ax.set_facecolor('#0D0D1A')
            ax.set_xlim(0, self._win_samples)
            ax.set_ylim(-32768, 32768)
            ax.set_yticks([-16384, 0, 16384])
            ax.set_yticklabels(['-50%', '0', '+50%'],
                               fontsize=6, color='#AAAACC')
            ax.set_xticklabels([])
            ax.tick_params(length=0)
            ax.set_ylabel(_CH_NAMES[i], fontsize=6.5,
                          color=_CH_COLOURS[i], labelpad=2)
            for spine in ax.spines.values():
                spine.set_color('#333355')
            ax.axhline(0, color='#333355', linewidth=0.5)
            self._ax_wave.append(ax)

        # FFT spectrum axis (row 4, left column)
        self._ax_fft = self._fig.add_subplot(gs[4, 0])
        self._ax_fft.set_facecolor('#0D0D1A')
        self._ax_fft.set_xlabel('Frequency (Hz)', fontsize=6, color='#AAAACC')
        self._ax_fft.set_ylabel('dB', fontsize=6, color='#AAAACC')
        self._ax_fft.tick_params(labelsize=6, colors='#AAAACC', length=2)
        for spine in self._ax_fft.spines.values():
            spine.set_color('#333355')
        self._ax_fft.set_title('FFT – CH 0', fontsize=7, color='#AAAACC', pad=2)

        # RMS meter axis (row 4, right column)
        self._ax_rms = self._fig.add_subplot(gs[4, 1])
        self._ax_rms.set_facecolor('#0D0D1A')
        self._ax_rms.set_xlabel('RMS level', fontsize=6, color='#AAAACC')
        self._ax_rms.set_xlim(0, 32768)
        self._ax_rms.set_yticks([0, 1, 2, 3])
        self._ax_rms.set_yticklabels(
            ['CH 0', 'CH 1', 'CH 2', 'CH 3'], fontsize=6, color='#AAAACC'
        )
        self._ax_rms.tick_params(length=0)
        for spine in self._ax_rms.spines.values():
            spine.set_color('#333355')
        self._ax_rms.set_title('RMS Levels', fontsize=7, color='#AAAACC', pad=2)

        # Title
        self._fig.suptitle(
            'Kinect v1  –  Microphone Array Monitor',
            fontsize=9, color='#DDDDFF', y=0.98,
        )

        # Pre-draw lines (we update their data each render for speed)
        x = np.arange(self._win_samples)
        self._wave_lines = []
        for i, ax in enumerate(self._ax_wave):
            (ln,) = ax.plot(x, np.zeros(self._win_samples),
                            color=_CH_COLOURS[i], linewidth=0.6, antialiased=True)
            self._wave_lines.append(ln)

        # FFT line (placeholder)
        freqs         = np.linspace(0, self._rate / 2, self._fft_bins // 2)
        self._fft_xdata = freqs
        (self._fft_line,) = self._ax_fft.plot(
            freqs, np.full(len(freqs), -80.0),
            color=_CH_COLOURS[0], linewidth=0.8
        )
        self._ax_fft.set_xlim(0, self._rate / 2)
        self._ax_fft.set_ylim(-80, 0)

        # RMS bars
        self._rms_bars = self._ax_rms.barh(
            [0, 1, 2, 3], [0, 0, 0, 0],
            color=_CH_COLOURS, height=0.55
        )

        plt.tight_layout(rect=[0, 0, 1, 0.96])

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _info_cb(self, msg: String) -> None:
        """Update audio format from the audio node's metadata."""
        try:
            meta = json.loads(msg.data)
            new_rate = meta.get('sample_rate', self._rate)
            new_ch   = meta.get('channels',    self._channels)

            if new_rate != self._rate or new_ch != self._channels:
                self._rate     = new_rate
                self._channels = new_ch
                new_win        = int(self._window_s * self._rate)
                if new_win != self._win_samples:
                    self._win_samples = new_win
                    for q in self._bufs:
                        q._maxlen = new_win     # type: ignore[attr-defined]
                self.get_logger().info(
                    f'Audio format updated: {self._rate} Hz, {self._channels} ch'
                )
        except Exception:
            pass

    def _raw_cb(self, msg: Int16MultiArray) -> None:
        """De-interleave samples into per-channel buffers and compute RMS."""
        all_samples = np.array(msg.data, dtype=np.int16)
        n_ch        = self._channels
        frames      = len(all_samples) // n_ch

        # De-interleave
        for ch in range(min(n_ch, 4)):
            ch_data = all_samples[ch::n_ch]
            self._bufs[ch].extend(ch_data.tolist())
            rms = float(np.sqrt(np.mean(ch_data.astype(np.float32) ** 2)))
            self._rms[ch] = rms

        self._chunks_received += 1

        # Publish RMS JSON on every audio chunk
        rms_json = json.dumps({
            'ch0': round(self._rms[0]),
            'ch1': round(self._rms[1]),
            'ch2': round(self._rms[2]),
            'ch3': round(self._rms[3]),
        })
        self._pub_rms.publish(String(data=rms_json))

    # ── Rendering ────────────────────────────────────────────────────────────

    def _render_and_publish(self) -> None:
        """Redraw the figure and publish it as a ROS Image."""
        try:
            t_start = time.time()
            self._update_figure()
            img_rgb = self._figure_to_numpy()
            ros_img = self._bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
            ros_img.header.stamp    = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'kinect_audio_frame'
            self._pub_viz.publish(ros_img)

            render_ms = (time.time() - t_start) * 1000
            self.get_logger().debug(
                f'Visualizer rendered in {render_ms:.0f} ms'
            )
        except Exception as exc:
            self.get_logger().warn(f'Render error: {exc}')

    def _update_figure(self) -> None:
        """Update all artists with current buffer data (no full redraw)."""
        win = self._win_samples

        # ── Waveforms ─────────────────────────────────────────────────────
        for i in range(4):
            buf  = np.array(self._bufs[i], dtype=np.float32)
            # Pad or trim to exactly win_samples
            if len(buf) < win:
                buf = np.pad(buf, (win - len(buf), 0))
            self._wave_lines[i].set_ydata(buf)

            # Autoscale Y per channel based on peak amplitude
            peak = max(float(np.abs(buf).max()), 100)
            margin = peak * 1.15
            self._ax_wave[i].set_ylim(-margin, margin)

        # ── FFT spectrum (channel 0) ──────────────────────────────────────
        ch0_buf = np.array(self._bufs[0], dtype=np.float32)
        if len(ch0_buf) >= self._fft_bins:
            window  = np.hanning(self._fft_bins)
            segment = ch0_buf[-self._fft_bins:] * window
            fft_mag = np.abs(np.fft.rfft(segment, n=self._fft_bins))
            # Convert to dB (add tiny floor to avoid log(0))
            fft_db  = 20 * np.log10(fft_mag[:self._fft_bins // 2] + 1e-6)
            # Normalise so 0 dB = max possible (32768 * fft_bins/2)
            ref_db  = 20 * np.log10(32768 * self._fft_bins / 2)
            fft_db -= ref_db
            fft_db  = np.clip(fft_db, -80, 0)
            self._fft_line.set_ydata(fft_db)

        # ── RMS bars ─────────────────────────────────────────────────────
        for i, bar in enumerate(self._rms_bars):
            bar.set_width(self._rms[i])
            # Colour-code: green → yellow → red based on level
            level = self._rms[i] / 32768
            if level < 0.5:
                bar.set_color(_CH_COLOURS[i])
            elif level < 0.8:
                bar.set_color('#F0C040')
            else:
                bar.set_color('#FF4040')

        # Timestamp in figure title
        elapsed = time.time() - getattr(self, '_t0_vis', time.time())
        if not hasattr(self, '_t0_vis'):
            self._t0_vis = time.time()

    def _figure_to_numpy(self) -> np.ndarray:
        """Render the figure to a (H, W, 3) uint8 RGB numpy array."""
        self._fig.canvas.draw()
        buf  = self._fig.canvas.buffer_rgba()
        arr  = np.asarray(buf, dtype=np.uint8)
        arr  = arr.reshape(self._fig.canvas.get_width_height()[::-1] + (4,))
        return arr[:, :, :3]           # drop alpha → RGB

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        plt.close(self._fig)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MicVisualizerNode.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
