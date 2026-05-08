#!/usr/bin/env python3
# Copyright 2024 ROS2 Community -- Apache-2.0
"""
ex08_speech_recognition_node.py  –  Offline/online speech recognition for Kinect v1.

Subscribes to /kinect/audio/mono (published by kinect_audio_node) and runs
speech recognition using the `speech_recognition` Python library, which
supports multiple backends:

  Backend         Mode      Notes
  ──────────────  ────────  ────────────────────────────────────────────────
  google          online    Free tier, requires internet, best accuracy
  vosk            offline   ~40 MB model, fast, good accuracy – RECOMMENDED
  sphinx          offline   No model download, lower accuracy, no internet

Voice Activity Detection (VAD) is built-in using an energy threshold so the
recognizer only processes audio that contains speech, avoiding wasted API
calls on silence.

Published topics
----------------
  /kinect/speech/text     (std_msgs/String)  – Raw recognised transcript
  /kinect/speech/command  (std_msgs/String)  – Matched robot command keyword
  /kinect/speech/status   (std_msgs/String)  – JSON: VAD state, word count, etc.
  /cmd_vel                (geometry_msgs/Twist) – Robot velocity command

Subscribed topics
-----------------
  /kinect/audio/mono      (std_msgs/Int16MultiArray)
  /kinect/audio/info      (std_msgs/String)  – JSON metadata from audio node

Parameters
----------
  backend             str    'vosk'   STT backend: 'vosk'|'google'|'sphinx'
  model_path          str    'vosk-model-small-en-us-0.15'
                             (only used when backend='vosk')
  sample_rate         int    16000
  buffer_duration_s   float  2.0     Seconds of audio to accumulate before STT
  vad_energy_thresh   int    300     RMS energy; chunks below = silence
  vad_silence_s       float  0.8     Silence duration that triggers recognition
  linear_speed        float  0.25    m/s
  angular_speed       float  0.50    rad/s
  command_timeout_s   float  3.0     Safety stop: zero vel if no cmd for N s

Vosk quick-start
----------------
  pip3 install vosk
  wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip vosk-model-small-en-us-0.15.zip
  ros2 run kinect_ros2 ex08_speech_recognition \\
      --ros-args -p model_path:=$PWD/vosk-model-small-en-us-0.15

Robot commands (speak any highlighted word)
-------------------------------------------
  forward | go | ahead | move    →  +X velocity
  back | backward | reverse      →  -X velocity
  left                           →  +Z rotation
  right                          →  -Z rotation
  stop | halt | freeze           →  zero velocity
  faster | speed up              →  +25% speed multiplier  (max 2.0×)
  slower | slow down             →  -25% speed multiplier  (min 0.25×)
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

# ── Command map: keyword → (linear_x_scale, angular_z_scale) ────────────────
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
    'abort':    ( 0.0,  0.0),
}
_FASTER_WORDS = ('faster', 'speed up', 'accelerate')
_SLOWER_WORDS = ('slower', 'slow down', 'decelerate')


class SpeechRecognitionNode(Node):
    """Subscribes to Kinect audio and publishes speech-driven robot commands."""

    def __init__(self) -> None:
        super().__init__('speech_recognition_node')

        if not _SR_OK:
            self.get_logger().fatal(
                'speech_recognition not installed.\n'
                '  pip3 install SpeechRecognition'
            )
            raise RuntimeError('SpeechRecognition unavailable')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('backend',           'vosk')
        self.declare_parameter('model_path',        'vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate',        16000)
        self.declare_parameter('buffer_duration_s',   2.0)
        self.declare_parameter('vad_energy_thresh',   300)
        self.declare_parameter('vad_silence_s',       0.8)
        self.declare_parameter('linear_speed',        0.25)
        self.declare_parameter('angular_speed',       0.50)
        self.declare_parameter('command_timeout_s',   3.0)

        p = self.get_parameter
        self._backend       = p('backend').value
        self._model_path    = p('model_path').value
        self._rate          = p('sample_rate').value
        self._buf_dur       = p('buffer_duration_s').value
        self._vad_thresh    = p('vad_energy_thresh').value
        self._vad_silence_s = p('vad_silence_s').value
        self._lin_spd       = p('linear_speed').value
        self._ang_spd       = p('angular_speed').value
        self._cmd_timeout   = p('command_timeout_s').value

        # ── Backend setup ──────────────────────────────────────────────────
        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold    = self._vad_thresh
        self._recognizer.dynamic_energy_threshold = False   # We manage it ourselves
        self._vosk_rec = None

        if self._backend == 'vosk':
            self._init_vosk()
        elif self._backend == 'google':
            self.get_logger().info('Using Google Web Speech (requires internet)')
        elif self._backend == 'sphinx':
            self.get_logger().info('Using CMU Sphinx (offline, lower accuracy)')
        else:
            self.get_logger().warn(
                f'Unknown backend "{self._backend}"; falling back to "sphinx"'
            )
            self._backend = 'sphinx'

        # ── Audio accumulation state ───────────────────────────────────────
        self._sample_rate      = self._rate         # may be updated from /info
        buf_samples            = int(self._buf_dur * self._rate)
        self._speech_buf: deque[int] = deque(maxlen=buf_samples)
        self._silence_buf: deque[int] = deque(
            maxlen=int(self._vad_silence_s * self._rate)
        )
        self._in_speech        = False
        self._last_speech_time = time.time()
        self._chunks_received  = 0

        # ── Robot state ────────────────────────────────────────────────────
        self._twist          = Twist()
        self._speed_mult     = 1.0
        self._last_cmd_time  = self.get_clock().now()
        self._total_cmds     = 0
        self._total_words    = 0

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_text    = self.create_publisher(String, '/kinect/speech/text',    10)
        self._pub_cmd     = self.create_publisher(String, '/kinect/speech/command', 10)
        self._pub_status  = self.create_publisher(String, '/kinect/speech/status',  10)
        self._pub_vel     = self.create_publisher(Twist,  '/cmd_vel',               10)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Int16MultiArray, '/kinect/audio/mono', self._audio_cb, 10
        )
        self.create_subscription(
            String, '/kinect/audio/info', self._info_cb, 10
        )

        # ── Timers ─────────────────────────────────────────────────────────
        self.create_timer(0.1,  self._publish_vel)        # 10 Hz cmd_vel
        self.create_timer(0.1,  self._safety_check)       # safety stop
        self.create_timer(5.0,  self._publish_status)     # diagnostics

        self._t0 = time.time()
        self._print_welcome()

    # ── Backend init ────────────────────────────────────────────────────────

    def _init_vosk(self) -> None:
        if not _VOSK_OK:
            self.get_logger().fatal(
                'vosk not installed.  Run:\n'
                '  pip3 install vosk\n'
                '  wget https://alphacephei.com/vosk/models/'
                'vosk-model-small-en-us-0.15.zip\n'
                '  unzip vosk-model-small-en-us-0.15.zip'
            )
            raise RuntimeError('vosk unavailable')
        self.get_logger().info(f'Loading Vosk model: "{self._model_path}" …')
        try:
            model          = Model(self._model_path)
            self._vosk_rec = KaldiRecognizer(model, self._rate)
            self._vosk_rec.SetWords(True)
            self.get_logger().info('Vosk model loaded ✓')
        except Exception as exc:
            self.get_logger().fatal(f'Failed to load Vosk model: {exc}')
            raise

    # ── Info callback ────────────────────────────────────────────────────────

    def _info_cb(self, msg: String) -> None:
        """Update sample rate from audio node metadata."""
        try:
            meta = json.loads(msg.data)
            self._sample_rate = meta.get('sample_rate', self._sample_rate)
        except Exception:
            pass

    # ── Audio callback ───────────────────────────────────────────────────────

    def _audio_cb(self, msg: Int16MultiArray) -> None:
        """Accumulate audio chunks with VAD; run STT when speech segment ends."""
        samples = np.array(msg.data, dtype=np.int16)
        self._chunks_received += 1

        # ── VAD: energy check ─────────────────────────────────────────────
        rms = int(np.sqrt(np.mean(samples.astype(np.int32) ** 2)))
        is_speech = rms > self._vad_thresh

        if is_speech:
            self._in_speech        = True
            self._last_speech_time = time.time()
            # Add to speech buffer
            self._speech_buf.extend(samples.tolist())
        else:
            # Track silence duration
            self._silence_buf.extend(samples.tolist())
            silence_elapsed = time.time() - self._last_speech_time

            if self._in_speech and silence_elapsed >= self._vad_silence_s:
                # Speech segment ended – run recognition
                speech_samples = list(self._speech_buf)
                if len(speech_samples) > self._rate * 0.3:   # ≥300 ms minimum
                    self._run_recognition(np.array(speech_samples, dtype=np.int16))
                self._speech_buf.clear()
                self._in_speech = False

    # ── Recognition ──────────────────────────────────────────────────────────

    def _run_recognition(self, samples: np.ndarray) -> None:
        """Run STT on a completed speech segment."""
        audio_bytes = samples.tobytes()
        text        = ''

        try:
            if self._backend == 'vosk' and self._vosk_rec is not None:
                # ── Vosk (offline) ───────────────────────────────────────
                self._vosk_rec.AcceptWaveform(audio_bytes)
                result = json.loads(self._vosk_rec.FinalResult())
                text   = result.get('text', '').strip().lower()

            else:
                # ── speech_recognition (Google / Sphinx) ─────────────────
                audio_data = sr.AudioData(audio_bytes, self._sample_rate, 2)
                if self._backend == 'google':
                    text = self._recognizer.recognize_google(audio_data).lower()
                else:
                    text = self._recognizer.recognize_sphinx(audio_data).lower()

        except sr.UnknownValueError:
            self.get_logger().debug('STT: speech not understood')
            return
        except sr.RequestError as exc:
            self.get_logger().error(f'STT API error: {exc}')
            return
        except Exception as exc:
            self.get_logger().warn(f'STT exception: {exc}')
            return

        if not text:
            return

        word_count = len(text.split())
        self._total_words += word_count
        self.get_logger().info(f'[STT] "{text}"  ({word_count} words)')

        # Publish raw transcript
        self._pub_text.publish(String(data=text))

        # Map to command
        self._process_command(text)

    # ── Command processing ───────────────────────────────────────────────────

    def _process_command(self, text: str) -> None:
        """Match recognised text to robot commands."""

        # Speed adjustments
        if any(w in text for w in _FASTER_WORDS):
            self._speed_mult = min(self._speed_mult + 0.25, 2.0)
            self.get_logger().info(f'Speed → {self._speed_mult:.2f}×')
            self._pub_cmd.publish(String(data=f'SPEED_{self._speed_mult:.2f}x'))
            return

        if any(w in text for w in _SLOWER_WORDS):
            self._speed_mult = max(self._speed_mult - 0.25, 0.25)
            self.get_logger().info(f'Speed → {self._speed_mult:.2f}×')
            self._pub_cmd.publish(String(data=f'SPEED_{self._speed_mult:.2f}x'))
            return

        # Movement commands (longest matching keyword wins)
        matched_kw  = None
        matched_lin = 0.0
        matched_ang = 0.0
        for kw, (lin, ang) in _COMMANDS.items():
            if kw in text:
                if matched_kw is None or len(kw) > len(matched_kw):
                    matched_kw  = kw
                    matched_lin = lin
                    matched_ang = ang

        if matched_kw is not None:
            self._twist           = Twist()
            self._twist.linear.x  = matched_lin * self._lin_spd * self._speed_mult
            self._twist.angular.z = matched_ang * self._ang_spd * self._speed_mult
            self._last_cmd_time   = self.get_clock().now()
            self._total_cmds     += 1
            self._pub_cmd.publish(String(data=matched_kw.upper()))
            self.get_logger().info(
                f'[CMD] {matched_kw.upper():10s}  '
                f'lin={self._twist.linear.x:+.2f} m/s  '
                f'ang={self._twist.angular.z:+.2f} rad/s  '
                f'({self._speed_mult:.2f}×)'
            )
        else:
            self.get_logger().debug(f'No command matched: "{text}"')

    # ── Periodic callbacks ───────────────────────────────────────────────────

    def _publish_vel(self) -> None:
        self._pub_vel.publish(self._twist)

    def _safety_check(self) -> None:
        """Zero velocity if no command received within timeout."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        is_moving = (
            abs(self._twist.linear.x) > 0.001 or
            abs(self._twist.angular.z) > 0.001
        )
        if elapsed > self._cmd_timeout and is_moving:
            self.get_logger().info(
                f'Safety stop: no command for {elapsed:.1f}s'
            )
            self._twist = Twist()

    def _publish_status(self) -> None:
        uptime = time.time() - self._t0
        status = {
            'uptime_s':       round(uptime, 1),
            'backend':        self._backend,
            'chunks_recv':    self._chunks_received,
            'total_words':    self._total_words,
            'total_cmds':     self._total_cmds,
            'speed_mult':     round(self._speed_mult, 2),
            'in_speech':      self._in_speech,
            'linear_x':       round(self._twist.linear.x, 3),
            'angular_z':      round(self._twist.angular.z, 3),
        }
        self._pub_status.publish(String(data=json.dumps(status)))

    # ── Welcome ──────────────────────────────────────────────────────────────

    def _print_welcome(self) -> None:
        self.get_logger().info(
            f'\n{"─"*56}\n'
            f' SpeechRecognitionNode ready\n'
            f'  Backend : {self._backend}\n'
            f'  VAD thr : {self._vad_thresh} RMS   '
            f'silence : {self._vad_silence_s}s\n'
            f'  Speed   : lin={self._lin_spd} m/s  '
            f'ang={self._ang_spd} rad/s\n'
            f'  Timeout : {self._cmd_timeout}s safety stop\n'
            f'{"─"*56}'
        )

    # ── Cleanup ──────────────────────────────────────────────────────────────

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
        node.get_logger().info('Shutting down SpeechRecognitionNode.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
