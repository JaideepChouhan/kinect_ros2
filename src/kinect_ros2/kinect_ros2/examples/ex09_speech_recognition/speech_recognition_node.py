#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
ex08_speech_recognition_node.py  –  Offline/online STT using Kinect hardware
                                     noise-cancelled microphone channel.

Subscribes to /kinect/audio/cancelled published by kinect_audio_cpp.
This channel is hardware beam-formed and noise-cancelled by the Kinect chip
itself (via libfreenect's cancelled_buffer) – the best possible signal for STT.

Each message is exactly 256 int16 samples at 16 kHz (~16 ms of audio).
The node buffers multiple messages using simple energy-based Voice Activity
Detection (VAD), then runs STT on complete speech segments.

Published topics
----------------
  /kinect/speech/text     (std_msgs/String)   – Raw recognised transcript
  /kinect/speech/command  (std_msgs/String)   – Matched command keyword
  /kinect/speech/status   (std_msgs/String)   – JSON diagnostics
  /cmd_vel                (geometry_msgs/Twist)

Subscribed topics
-----------------
  /kinect/audio/cancelled (std_msgs/Int16MultiArray)  ← from kinect_audio_cpp
  /kinect/audio/info      (std_msgs/String)

Parameters
----------
  backend             str    vosk     vosk | google | sphinx
  model_path          str    vosk-model-small-en-us-0.15
  sample_rate         int    16000
  vad_energy_thresh   int    300      RMS energy above which = speech
  vad_silence_s       float  0.8      Silence after speech → trigger STT
  linear_speed        float  0.25     m/s
  angular_speed       float  0.50     rad/s
  command_timeout_s   float  3.0      Safety-stop if no command for N s

Vosk quick-start
----------------
  pip3 install vosk
  wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip vosk-model-small-en-us-0.15.zip
  ros2 run kinect_ros2 ex08_speech_recognition \\
      --ros-args -p model_path:=$PWD/vosk-model-small-en-us-0.15
"""

import json
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String
from geometry_msgs.msg import Twist

try:
    import speech_recognition as sr
    _SR_OK = True
except ImportError:
    _SR_OK = False

try:
    from vosk import Model, KaldiRecognizer
    _VOSK_OK = True
except ImportError:
    _VOSK_OK = False

# ── Command table ────────────────────────────────────────────────────────────
_COMMANDS: dict[str, tuple[float, float]] = {
    'forward':  ( 1.0,  0.0),
    'go':       ( 1.0,  0.0),
    'ahead':    ( 1.0,  0.0),
    'move':     ( 1.0,  0.0),
    'advance':  ( 1.0,  0.0),
    'back':     (-1.0,  0.0),
    'backward': (-1.0,  0.0),
    'reverse':  (-1.0,  0.0),
    'left':     ( 0.0,  1.0),
    'right':    ( 0.0, -1.0),
    'stop':     ( 0.0,  0.0),
    'halt':     ( 0.0,  0.0),
    'freeze':   ( 0.0,  0.0),
    'cancel':   ( 0.0,  0.0),
}
_FASTER = ('faster', 'speed up', 'accelerate')
_SLOWER = ('slower', 'slow down', 'decelerate')

# Samples per message – fixed by kinect_audio_cpp from audio.c
SAMPLES_PER_MSG = 256
SAMPLE_RATE     = 16000


class SpeechRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')

        if not _SR_OK:
            self.get_logger().fatal('pip3 install SpeechRecognition')
            raise RuntimeError('SpeechRecognition unavailable')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('backend',           'vosk')
        self.declare_parameter('model_path',        'vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate',        SAMPLE_RATE)
        self.declare_parameter('vad_energy_thresh',  300)
        self.declare_parameter('vad_silence_s',      0.8)
        self.declare_parameter('linear_speed',       0.25)
        self.declare_parameter('angular_speed',      0.50)
        self.declare_parameter('command_timeout_s',  3.0)

        p = self.get_parameter
        self._backend       = p('backend').value
        self._model_path    = p('model_path').value
        self._rate          = p('sample_rate').value
        self._vad_thresh    = p('vad_energy_thresh').value
        self._vad_silence   = p('vad_silence_s').value
        self._lin_spd       = p('linear_speed').value
        self._ang_spd       = p('angular_speed').value
        self._cmd_timeout   = p('command_timeout_s').value

        # ── STT backend ────────────────────────────────────────────────────
        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold           = self._vad_thresh
        self._recognizer.dynamic_energy_threshold   = False
        self._vosk_rec = None

        if self._backend == 'vosk':
            self._init_vosk()
        else:
            self.get_logger().info(f'STT backend: {self._backend}')

        # ── Audio accumulation state ───────────────────────────────────────
        # Buffer enough for 5 s of speech (worst case)
        max_samples        = int(5.0 * self._rate)
        self._speech_buf: deque[int] = deque(maxlen=max_samples)
        self._in_speech    = False
        self._last_speech  = time.time()
        self._chunks_recv  = 0

        # ── Robot state ────────────────────────────────────────────────────
        self._twist         = Twist()
        self._speed_mult    = 1.0
        self._last_cmd_t    = self.get_clock().now()
        self._total_cmds    = 0
        self._total_words   = 0

        # ── Publishers / Subscribers ────────────────────────────────────────
        self._pub_text   = self.create_publisher(String, '/kinect/speech/text',    10)
        self._pub_cmd    = self.create_publisher(String, '/kinect/speech/command', 10)
        self._pub_status = self.create_publisher(String, '/kinect/speech/status',  10)
        self._pub_vel    = self.create_publisher(Twist,  '/cmd_vel',               10)

        # CHANGED: subscribe to /kinect/audio/cancelled (Int16MultiArray)
        # published by kinect_audio_cpp using libfreenect's cancelled_buffer.
        # This is the hardware noise-cancelled channel – ideal for STT.
        self.create_subscription(
            Int16MultiArray,
            '/kinect/audio/cancelled',   # ← from kinect_audio_cpp C++ node
            self._audio_cb,
            10,
        )
        self.create_subscription(String, '/kinect/audio/info', self._info_cb, 10)

        # ── Timers ─────────────────────────────────────────────────────────
        self.create_timer(0.1, self._publish_vel)
        self.create_timer(0.1, self._safety_check)
        self.create_timer(5.0, self._publish_status)

        self._t0 = time.time()
        self.get_logger().info(
            f'\n{"─"*54}\n'
            f' SpeechRecognitionNode  –  backend: {self._backend}\n'
            f'  Listening on /kinect/audio/cancelled (HW noise-cancelled)\n'
            f'  VAD thresh={self._vad_thresh} RMS  silence={self._vad_silence}s\n'
            f'  Speed: lin={self._lin_spd}m/s  ang={self._ang_spd}rad/s\n'
            f'{"─"*54}'
        )

    # ── Vosk init ────────────────────────────────────────────────────────────

    def _init_vosk(self) -> None:
        if not _VOSK_OK:
            self.get_logger().fatal(
                'vosk not installed.\n'
                '  pip3 install vosk\n'
                '  wget https://alphacephei.com/vosk/models/'
                'vosk-model-small-en-us-0.15.zip\n'
                '  unzip vosk-model-small-en-us-0.15.zip'
            )
            raise RuntimeError('vosk unavailable')

        self.get_logger().info(f'Loading Vosk model: {self._model_path} …')
        model = Model(self._model_path)
        self._vosk_rec = KaldiRecognizer(model, self._rate)
        self._vosk_rec.SetWords(True)
        self.get_logger().info('Vosk model loaded ✓')

    # ── Info callback ────────────────────────────────────────────────────────

    def _info_cb(self, msg: String) -> None:
        try:
            meta = json.loads(msg.data)
            self._rate = meta.get('sample_rate', self._rate)
        except Exception:
            pass

    # ── Audio callback ───────────────────────────────────────────────────────
    # Each message = 256 int16 samples at 16 kHz (~16 ms)
    # from the libfreenect cancelled_buffer (hardware noise-cancelled).

    def _audio_cb(self, msg: Int16MultiArray) -> None:
        samples = np.array(msg.data, dtype=np.int16)
        self._chunks_recv += 1

        rms      = int(np.sqrt(np.mean(samples.astype(np.int32) ** 2)))
        is_voice = rms > self._vad_thresh

        if is_voice:
            self._in_speech   = True
            self._last_speech = time.time()
            self._speech_buf.extend(samples.tolist())
        else:
            silence_s = time.time() - self._last_speech
            if self._in_speech and silence_s >= self._vad_silence:
                segment = np.array(list(self._speech_buf), dtype=np.int16)
                if len(segment) >= int(self._rate * 0.2):   # min 200 ms
                    self._run_stt(segment)
                self._speech_buf.clear()
                self._in_speech = False

    # ── STT ─────────────────────────────────────────────────────────────────

    def _run_stt(self, samples: np.ndarray) -> None:
        audio_bytes = samples.tobytes()
        text = ''
        try:
            if self._backend == 'vosk' and self._vosk_rec is not None:
                self._vosk_rec.AcceptWaveform(audio_bytes)
                result = json.loads(self._vosk_rec.FinalResult())
                text   = result.get('text', '').strip().lower()
            else:
                audio_data = sr.AudioData(audio_bytes, self._rate, 2)
                if self._backend == 'google':
                    text = self._recognizer.recognize_google(audio_data).lower()
                else:
                    text = self._recognizer.recognize_sphinx(audio_data).lower()
        except sr.UnknownValueError:
            return
        except Exception as exc:
            self.get_logger().warn(f'STT error: {exc}')
            return

        if not text:
            return

        self._total_words += len(text.split())
        self.get_logger().info(f'[STT] "{text}"')
        self._pub_text.publish(String(data=text))
        self._process_command(text)

    # ── Command processing ───────────────────────────────────────────────────

    def _process_command(self, text: str) -> None:
        if any(w in text for w in _FASTER):
            self._speed_mult = min(self._speed_mult + 0.25, 2.0)
            self._pub_cmd.publish(String(data=f'SPEED_{self._speed_mult:.2f}x'))
            return
        if any(w in text for w in _SLOWER):
            self._speed_mult = max(self._speed_mult - 0.25, 0.25)
            self._pub_cmd.publish(String(data=f'SPEED_{self._speed_mult:.2f}x'))
            return

        matched = None
        for kw, (lin, ang) in _COMMANDS.items():
            if kw in text:
                if matched is None or len(kw) > len(matched[0]):
                    matched = (kw, lin, ang)

        if matched:
            kw, lin, ang = matched
            self._twist           = Twist()
            self._twist.linear.x  = lin * self._lin_spd * self._speed_mult
            self._twist.angular.z = ang * self._ang_spd * self._speed_mult
            self._last_cmd_t      = self.get_clock().now()
            self._total_cmds     += 1
            self._pub_cmd.publish(String(data=kw.upper()))
            self.get_logger().info(
                f'[CMD] {kw.upper():10s}  '
                f'lin={self._twist.linear.x:+.2f}  '
                f'ang={self._twist.angular.z:+.2f}  '
                f'({self._speed_mult:.2f}×)'
            )

    # ── Periodic ─────────────────────────────────────────────────────────────

    def _publish_vel(self) -> None:
        self._pub_vel.publish(self._twist)

    def _safety_check(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
        moving  = abs(self._twist.linear.x) > 0.001 or abs(self._twist.angular.z) > 0.001
        if elapsed > self._cmd_timeout and moving:
            self.get_logger().info(f'Safety stop ({elapsed:.1f}s no command)')
            self._twist = Twist()

    def _publish_status(self) -> None:
        self._pub_status.publish(String(data=json.dumps({
            'uptime_s':     round(time.time() - self._t0, 1),
            'backend':      self._backend,
            'chunks_recv':  self._chunks_recv,
            'total_words':  self._total_words,
            'total_cmds':   self._total_cmds,
            'speed_mult':   round(self._speed_mult, 2),
            'in_speech':    self._in_speech,
        })))

    def destroy_node(self) -> None:
        try:
            self._pub_vel.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
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

