#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
kinect_audio_node.py  –  ROS 2 Jazzy audio driver for the Kinect v1 mic array.

IMPORTANT – How the Kinect mic actually works on Linux
------------------------------------------------------
The Kinect v1 microphone array does NOT use libfreenect for audio.
It presents itself to the Linux kernel as a standard USB Audio Class device
("Xbox NUI Audio") accessible through ALSA and PyAudio like any USB mic.
The libfreenect Python bindings have NO audio API – any code calling
freenect.get_audio() / freenect.start_audio() will raise AttributeError.

The array has 4 physical microphones arranged in a horizontal line.
When opened with channels=4 the samples arrive interleaved per frame:
    [ch0_s0, ch1_s0, ch2_s0, ch3_s0,  ch0_s1, ch1_s1, ch2_s1, ch3_s1, ...]
ch0 = left mic, ch3 = right mic.  Hardware beam-forming is NOT available;
all 4 raw signals are published so downstream nodes can beam-form in software.

Published topics
----------------
  /kinect/audio/raw   (std_msgs/Int16MultiArray)
      All-channel interleaved PCM int16.
      layout.dim[0]  label="frames"    size=chunk_frames
      layout.dim[1]  label="channels"  size=channels (4)
      Total len = chunk_frames × channels

  /kinect/audio/mono  (std_msgs/Int16MultiArray)
      Channel-0 only – suitable for speech recognition.
      layout.dim[0]  label="samples"   size=chunk_frames

  /kinect/audio/info  (std_msgs/String, latched-like)
      JSON published repeatedly for ~2 s on startup so late subscribers
      receive it:
      {"device_index":N, "device_name":"...", "sample_rate":16000,
       "channels":4, "chunk_frames":4096}

Parameters
----------
  device_index   int    -1      Auto-detect Kinect mic; set ≥0 to force
  sample_rate    int  16000     Fixed by Kinect hardware; do not change
  chunk_frames   int   4096     PCM frames per ROS msg (~256 ms @ 16 kHz)
  channels       int      4     4 = full array; 1 = mono-only (lower CPU)
"""

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout, String

try:
    import pyaudio
    _PYAUDIO_OK = True
except ImportError:
    _PYAUDIO_OK = False

_KINECT_AUDIO_KEYWORDS = ('xbox nui audio', 'kinect', 'nui audio', 'xbox audio')


class KinectAudioNode(Node):
    """Publishes raw PCM audio from the Kinect v1 four-microphone array."""

    def __init__(self) -> None:
        super().__init__('kinect_audio_node')

        if not _PYAUDIO_OK:
            self.get_logger().fatal(
                'pyaudio not available.  Install with:\n'
                '  sudo apt install portaudio19-dev\n'
                '  pip3 install pyaudio'
            )
            raise RuntimeError('pyaudio unavailable')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('device_index',  -1)
        self.declare_parameter('sample_rate',  16000)
        self.declare_parameter('chunk_frames', 4096)
        self.declare_parameter('channels',        4)

        p = self.get_parameter
        self._dev_idx      = p('device_index').value
        self._rate         = p('sample_rate').value
        self._chunk_frames = p('chunk_frames').value
        self._channels     = p('channels').value

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_raw  = self.create_publisher(Int16MultiArray, '/kinect/audio/raw',  10)
        self._pub_mono = self.create_publisher(Int16MultiArray, '/kinect/audio/mono', 10)
        self._pub_info = self.create_publisher(String,          '/kinect/audio/info', 10)

        # ── PyAudio setup ──────────────────────────────────────────────────
        self._pa = pyaudio.PyAudio()

        if self._dev_idx < 0:
            self._dev_idx = self._find_kinect_mic()

        dev_info      = self._pa.get_device_info_by_index(self._dev_idx)
        dev_name      = dev_info['name']
        max_ch        = int(dev_info['maxInputChannels'])

        if self._channels > max_ch:
            self.get_logger().warn(
                f'Requested channels={self._channels} > device max={max_ch}. '
                f'Clamped to {max_ch}.'
            )
            self._channels = max_ch

        self.get_logger().info(
            f'Kinect audio device  ▸  [{self._dev_idx}] "{dev_name}"\n'
            f'  rate={self._rate} Hz  channels={self._channels}  '
            f'chunk={self._chunk_frames} frames '
            f'({self._chunk_frames / self._rate * 1000:.0f} ms)'
        )

        # Latched info (republish for 2 s so late subscribers catch it)
        self._info_json    = json.dumps({
            'device_index': self._dev_idx,
            'device_name':  dev_name,
            'sample_rate':  self._rate,
            'channels':     self._channels,
            'chunk_frames': self._chunk_frames,
        })
        self._info_repeats = 20
        self._info_timer   = self.create_timer(0.1, self._pub_info_latch)

        # ── Audio stream ───────────────────────────────────────────────────
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=self._channels,
            rate=self._rate,
            input=True,
            input_device_index=self._dev_idx,
            frames_per_buffer=self._chunk_frames,
        )

        # Capture slightly faster than one chunk to avoid buffer lag
        self._timer = self.create_timer(
            self._chunk_frames / self._rate * 0.95, self._capture
        )
        self._frame_count = 0
        self._t0          = time.time()

        self.get_logger().info(
            'KinectAudioNode started.\n'
            '  /kinect/audio/raw   – all channels interleaved\n'
            '  /kinect/audio/mono  – channel 0 (left mic)\n'
            '  /kinect/audio/info  – JSON metadata'
        )

    # ── Device discovery ────────────────────────────────────────────────────

    def _find_kinect_mic(self) -> int:
        self.get_logger().info('Scanning audio input devices for Kinect mic …')
        inputs = []
        for i in range(self._pa.get_device_count()):
            info = self._pa.get_device_info_by_index(i)
            if int(info['maxInputChannels']) < 1:
                continue
            name = info['name']
            inputs.append((i, name))
            self.get_logger().info(
                f'  [{i}] "{name}"  ({info["maxInputChannels"]} ch)'
            )
            if any(kw in name.lower() for kw in _KINECT_AUDIO_KEYWORDS):
                self.get_logger().info('      ↑ matched Kinect mic keywords')
                return i

        if len(inputs) == 1:
            idx, name = inputs[0]
            self.get_logger().warn(
                f'Kinect keyword not matched; single input device found.\n'
                f'  Using [{idx}] "{name}".  '
                f'Pass device_index:=N to force a specific device.'
            )
            return idx

        raise RuntimeError(
            'Kinect mic not auto-detected.  '
            'Check: lsusb | grep -i kinect  and  arecord -l\n'
            'Then set the device_index parameter manually.'
        )

    # ── Info latch ──────────────────────────────────────────────────────────

    def _pub_info_latch(self) -> None:
        self._pub_info.publish(String(data=self._info_json))
        self._info_repeats -= 1
        if self._info_repeats <= 0:
            self._info_timer.cancel()

    # ── Capture callback ────────────────────────────────────────────────────

    def _capture(self) -> None:
        try:
            raw_bytes = self._stream.read(
                self._chunk_frames, exception_on_overflow=False
            )
        except Exception as exc:
            self.get_logger().warn(f'Audio read failed: {exc}')
            return

        all_samples = np.frombuffer(raw_bytes, dtype=np.int16)

        # ── Publish raw (interleaved, all channels) ──────────────────────
        raw_msg = Int16MultiArray(
            layout=MultiArrayLayout(
                dim=[
                    MultiArrayDimension(
                        label='frames',
                        size=self._chunk_frames,
                        stride=self._chunk_frames * self._channels,
                    ),
                    MultiArrayDimension(
                        label='channels',
                        size=self._channels,
                        stride=self._channels,
                    ),
                ],
                data_offset=0,
            ),
            data=all_samples.tolist(),
        )
        self._pub_raw.publish(raw_msg)

        # ── Publish mono (channel 0 de-interleaved) ──────────────────────
        mono = all_samples[0::self._channels] if self._channels > 1 else all_samples
        mono_msg = Int16MultiArray(
            layout=MultiArrayLayout(
                dim=[MultiArrayDimension(label='samples', size=len(mono), stride=len(mono))],
                data_offset=0,
            ),
            data=mono.tolist(),
        )
        self._pub_mono.publish(mono_msg)

        self._frame_count += 1
        if self._frame_count % 50 == 0:
            elapsed     = time.time() - self._t0
            recorded_s  = self._frame_count * self._chunk_frames / self._rate
            self.get_logger().info(
                f'Audio ▸ {self._frame_count} chunks  '
                f'{recorded_s:.0f} s total  '
                f'{self._frame_count / elapsed:.1f} chunks/s'
            )

    # ── Cleanup ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if hasattr(self, '_stream'):
            try:
                if self._stream.is_active():
                    self._stream.stop_stream()
                self._stream.close()
            except Exception:
                pass
        if hasattr(self, '_pa'):
            try:
                self._pa.terminate()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinectAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down KinectAudioNode.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
