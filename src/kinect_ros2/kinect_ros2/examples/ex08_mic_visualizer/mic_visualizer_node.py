#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
ex09_mic_visualizer_node.py  –  Real-time visualisation of all Kinect mic channels.

Subscribes to the two topics published by kinect_audio_cpp (C++ libfreenect node):

  /kinect/audio/raw       (Int32MultiArray)  –  4 physical mic channels
  /kinect/audio/cancelled (Int16MultiArray)  –  hardware noise-cancelled channel

From audio.c: 256 samples per message at 16 kHz → callbacks at ~62.5 Hz.
The int32 mic values are ~24-bit ADC data (range roughly ±2²³).
The int16 cancelled channel is the beam-formed output (range ±2¹⁵).

Figure layout  (640 × 540 px, RGB8):
┌─────────────────────────────────────────────────────────┐
│  MIC 0 ─── int32 waveform (blue)                        │
│  MIC 1 ─── int32 waveform (orange)                      │
│  MIC 2 ─── int32 waveform (green)                       │
│  MIC 3 ─── int32 waveform (red)                         │
│  CANCELLED ─ int16 waveform (cyan, HW noise-cancelled)  │
├────────────────────────────┬────────────────────────────┤
│  FFT – cancelled channel   │  RMS level bars (all 5 ch) │
└────────────────────────────┴────────────────────────────┘

Published topics
----------------
  /kinect/audio/visualizer (sensor_msgs/Image rgb8)  ~5 Hz
  /kinect/audio/rms        (std_msgs/String)  JSON per audio chunk

Subscribed topics
-----------------
  /kinect/audio/raw       (std_msgs/Int32MultiArray)  ← kinect_audio_cpp
  /kinect/audio/cancelled (std_msgs/Int16MultiArray)  ← kinect_audio_cpp
  /kinect/audio/info      (std_msgs/String)

Parameters
----------
  window_s       float  1.0    Rolling waveform duration (seconds)
  render_rate_hz float  5.0    Publish rate for the image
  figure_width   int    640    Output image width  (px)
  figure_height  int    540    Output image height (px)
  fft_bins       int    512    FFT resolution
"""

import json
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int16MultiArray, String
from cv_bridge import CvBridge

import matplotlib
matplotlib.use('Agg')                       # headless, no display needed
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Channel styling ──────────────────────────────────────────────────────────
_CH_COLOURS = ['#4C8EF0', '#F07A2E', '#3EBD6F', '#E35C5C', '#00CCCC']
_CH_LABELS  = ['MIC 0 (int32)', 'MIC 1 (int32)', 'MIC 2 (int32)',
               'MIC 3 (int32)', 'CANCELLED (int16)']

SAMPLES_PER_MSG = 256
SAMPLE_RATE_HZ  = 16000


class MicVisualizerNode(Node):
    """Real-time composite mic-array diagnostic image publisher."""

    def __init__(self) -> None:
        super().__init__('mic_visualizer_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('window_s',       1.0)
        self.declare_parameter('render_rate_hz', 5.0)
        self.declare_parameter('figure_width',   640)
        self.declare_parameter('figure_height',  540)
        self.declare_parameter('fft_bins',       512)

        p = self.get_parameter
        self._window_s    = p('window_s').value
        self._render_rate = p('render_rate_hz').value
        self._fig_w       = p('figure_width').value
        self._fig_h       = p('figure_height').value
        self._fft_bins    = p('fft_bins').value

        # ── Audio state ─────────────────────────────────────────────────────
        self._rate     = SAMPLE_RATE_HZ
        self._win_n    = int(self._window_s * self._rate)

        # 5 rolling buffers: mic0–mic3 (int32 range) + cancelled (int16 range)
        self._bufs: list[deque] = [
            deque([0] * self._win_n, maxlen=self._win_n) for _ in range(5)
        ]
        self._rms = [0.0] * 5

        # ── cv_bridge ───────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_viz = self.create_publisher(Image,  '/kinect/audio/visualizer', 10)
        self._pub_rms = self.create_publisher(String, '/kinect/audio/rms',        10)

        # ── Subscribers ─────────────────────────────────────────────────────
        # Raw 4-channel int32 from libfreenect mic_buffer[0..3]
        self.create_subscription(
            Int32MultiArray,
            '/kinect/audio/raw',          # ← kinect_audio_cpp C++ node
            self._raw_cb,
            10,
        )
        # Noise-cancelled int16 from libfreenect cancelled_buffer
        self.create_subscription(
            Int16MultiArray,
            '/kinect/audio/cancelled',    # ← kinect_audio_cpp C++ node
            self._cancelled_cb,
            10,
        )
        self.create_subscription(String, '/kinect/audio/info', self._info_cb, 10)

        # ── Build figure ─────────────────────────────────────────────────────
        self._setup_figure()

        # ── Render timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self._render_rate, self._render_and_publish)

        self._chunks_raw  = 0
        self._chunks_canc = 0

        self.get_logger().info(
            f'MicVisualizerNode ready\n'
            f'  /kinect/audio/raw       → 4 × int32 waveforms\n'
            f'  /kinect/audio/cancelled → 1 × int16 waveform + FFT\n'
            f'  /kinect/audio/visualizer← {self._fig_w}×{self._fig_h}px '
            f'@ {self._render_rate:.0f} Hz'
        )

    # ── Figure setup ────────────────────────────────────────────────────────

    def _setup_figure(self) -> None:
        dpi      = 100
        self._fig = plt.figure(
            figsize=(self._fig_w / dpi, self._fig_h / dpi),
            dpi=dpi,
            facecolor='#12121E',
        )

        # 5 waveform rows + 1 bottom row (FFT | RMS)
        gs = gridspec.GridSpec(
            6, 2,
            figure=self._fig,
            hspace=0.55, wspace=0.35,
            left=0.09, right=0.97,
            top=0.94, bottom=0.06,
        )

        # ── Waveform axes (rows 0–4, full width) ──────────────────────────
        self._ax_wave = []
        self._wave_lines = []
        x = np.arange(self._win_n)

        for i in range(5):
            ax = self._fig.add_subplot(gs[i, :])
            ax.set_facecolor('#0A0A14')
            ax.set_xlim(0, self._win_n)
            ax.set_xticklabels([])
            ax.tick_params(length=0, labelsize=0)
            ax.set_ylabel(_CH_LABELS[i], fontsize=6,
                          color=_CH_COLOURS[i], labelpad=2)
            for sp in ax.spines.values():
                sp.set_color('#222240')
            ax.axhline(0, color='#222240', linewidth=0.4)

            (ln,) = ax.plot(x, np.zeros(self._win_n),
                            color=_CH_COLOURS[i], linewidth=0.6, antialiased=True)
            self._ax_wave.append(ax)
            self._wave_lines.append(ln)

        # ── FFT axis (row 5, left column) ─────────────────────────────────
        self._ax_fft = self._fig.add_subplot(gs[5, 0])
        self._ax_fft.set_facecolor('#0A0A14')
        self._ax_fft.set_xlabel('Frequency (Hz)', fontsize=6, color='#AAAACC')
        self._ax_fft.set_ylabel('dB', fontsize=6, color='#AAAACC')
        self._ax_fft.tick_params(labelsize=5, colors='#AAAACC', length=2)
        for sp in self._ax_fft.spines.values():
            sp.set_color('#222240')
        self._ax_fft.set_xlim(0, self._rate / 2)
        self._ax_fft.set_ylim(-80, 0)
        self._ax_fft.set_title('FFT – CANCELLED ch (int16)', fontsize=6,
                               color='#AAAACC', pad=2)

        freqs = np.linspace(0, self._rate / 2, self._fft_bins // 2)
        (self._fft_line,) = self._ax_fft.plot(
            freqs, np.full(len(freqs), -80.0),
            color=_CH_COLOURS[4], linewidth=0.7,
        )

        # ── RMS axis (row 5, right column) ────────────────────────────────
        self._ax_rms = self._fig.add_subplot(gs[5, 1])
        self._ax_rms.set_facecolor('#0A0A14')
        self._ax_rms.set_title('RMS Levels', fontsize=6, color='#AAAACC', pad=2)
        self._ax_rms.set_xlim(0, 1.0)   # normalised 0–1
        self._ax_rms.set_yticks([0, 1, 2, 3, 4])
        self._ax_rms.set_yticklabels(
            ['M0', 'M1', 'M2', 'M3', 'CAN'], fontsize=5, color='#AAAACC'
        )
        self._ax_rms.tick_params(length=0)
        for sp in self._ax_rms.spines.values():
            sp.set_color('#222240')

        self._rms_bars = self._ax_rms.barh(
            [0, 1, 2, 3, 4], [0.0] * 5,
            color=_CH_COLOURS, height=0.55,
        )

        # Title
        self._fig.suptitle(
            'Kinect v1  –  Microphone Array  (libfreenect C API)',
            fontsize=8, color='#DDDDFF', y=0.985,
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _info_cb(self, msg: String) -> None:
        try:
            meta = json.loads(msg.data)
            new_rate = meta.get('sample_rate', self._rate)
            if new_rate != self._rate:
                self._rate  = new_rate
                self._win_n = int(self._window_s * self._rate)
                for q in self._bufs:
                    q._maxlen = self._win_n   # type: ignore[attr-defined]
        except Exception:
            pass

    def _raw_cb(self, msg: Int32MultiArray) -> None:
        """De-interleave 4 int32 mic channels from layout [frames × channels]."""
        all_s  = np.array(msg.data, dtype=np.int32)  # shape: (256*4,)
        frames = len(all_s) // 4

        # Normalise int32 → float in [-1, 1] using 24-bit ADC range
        _INT32_NORM = float(2**23)

        for ch in range(4):
            ch_data = all_s[ch::4][:frames]           # de-interleave channel ch
            self._bufs[ch].extend(ch_data.tolist())
            # RMS normalised 0–1
            self._rms[ch] = float(
                np.sqrt(np.mean((ch_data.astype(np.float64) / _INT32_NORM) ** 2))
            )

        self._chunks_raw += 1
        self._pub_rms.publish(String(data=json.dumps({
            'mic0': round(self._rms[0], 4),
            'mic1': round(self._rms[1], 4),
            'mic2': round(self._rms[2], 4),
            'mic3': round(self._rms[3], 4),
            'cancelled': round(self._rms[4], 4),
        })))

    def _cancelled_cb(self, msg: Int16MultiArray) -> None:
        """Buffer the hardware noise-cancelled int16 channel."""
        samples = np.array(msg.data, dtype=np.int16)
        self._bufs[4].extend(samples.tolist())
        rms_norm       = float(np.sqrt(np.mean((samples.astype(np.float32) / 32768) ** 2)))
        self._rms[4]   = rms_norm
        self._chunks_canc += 1

    # ── Rendering ────────────────────────────────────────────────────────────

    def _render_and_publish(self) -> None:
        try:
            self._update_figure()
            img = self._figure_to_rgb()
            ros_img = self._bridge.cv2_to_imgmsg(img, encoding='rgb8')
            ros_img.header.stamp    = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'kinect_audio_frame'
            self._pub_viz.publish(ros_img)
        except Exception as exc:
            self.get_logger().warn(f'Render error: {exc}')

    def _update_figure(self) -> None:
        """Update all matplotlib artists in-place (fast, no full redraw)."""
        _INT32_NORM = float(2**23)

        for i in range(5):
            buf = np.array(list(self._bufs[i]), dtype=np.float64)
            if len(buf) < self._win_n:
                buf = np.pad(buf, (self._win_n - len(buf), 0))

            # Normalise for display
            if i < 4:
                buf = buf / _INT32_NORM   # int32 mic → [-1, 1]
            else:
                buf = buf / 32768.0       # int16 cancelled → [-1, 1]

            self._wave_lines[i].set_ydata(buf)
            peak = max(float(np.abs(buf).max()), 1e-6)
            self._ax_wave[i].set_ylim(-peak * 1.2, peak * 1.2)

        # FFT on cancelled channel (buffer index 4)
        canc_buf = np.array(list(self._bufs[4]), dtype=np.float32)
        if len(canc_buf) >= self._fft_bins:
            seg    = canc_buf[-self._fft_bins:] / 32768.0
            window = np.hanning(self._fft_bins)
            mag    = np.abs(np.fft.rfft(seg * window, n=self._fft_bins))
            ref    = self._fft_bins / 2.0
            db     = np.clip(20 * np.log10(mag[:self._fft_bins // 2] / (ref + 1e-10)), -80, 0)
            self._fft_line.set_ydata(db)

        # RMS bars (normalised 0–1)
        for i, bar in enumerate(self._rms_bars):
            v = min(self._rms[i], 1.0)
            bar.set_width(v)
            bar.set_color('#3EBD6F' if v < 0.5 else '#F0C040' if v < 0.8 else '#FF4040')

    def _figure_to_rgb(self) -> np.ndarray:
        self._fig.canvas.draw()
        buf = self._fig.canvas.buffer_rgba()
        arr = np.asarray(buf, dtype=np.uint8)
        arr = arr.reshape(self._fig.canvas.get_width_height()[::-1] + (4,))
        return arr[:, :, :3]   # drop alpha → RGB

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
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

