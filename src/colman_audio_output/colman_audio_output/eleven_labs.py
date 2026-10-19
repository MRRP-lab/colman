#!/usr/bin/env python3
"""ROS 2 node: subscribes to LLM text output and speaks it with ElevenLabs TTS.

Subscribes:  <input_topic>  (std_msgs/String)  -- text from the LLM node
Publishes:   tts_speaking   (std_msgs/Bool)    -- True while audio is playing
                                                  (handy for muting the mic node)

Run with:  ros2 run colman_audio_output eleven_labs
Requires ELEVENLABS_API_KEY in the environment, plus mpv (stream mode) or
ffplay from ffmpeg (play mode) on the PATH. All three come with the pixi env.
"""

import hashlib
import os
import queue
import shutil
import threading

import rclpy
from elevenlabs.client import ElevenLabs
from elevenlabs.play import play, save, stream
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

# External player each mode shells out to (save mode needs none).
PLAYERS = {"stream": "mpv", "play": "ffplay", "save": None}


class ElevenLabsTTSNode(Node):
    def __init__(self):
        super().__init__("elevenlabs_tts")

        # ---- Parameters ----
        self.declare_parameter("input_topic", "llm_response")
        self.declare_parameter("voice", "Doug VO Only")  # voice name or voice_id
        self.declare_parameter("model", "eleven_flash_v2")
        self.declare_parameter("mode", "stream")  # stream | play | save
        self.declare_parameter("output_dir", "/tmp/tts")  # used when mode == save
        self.declare_parameter("api_key_env", "ELEVENLABS_API_KEY")
        self.declare_parameter("max_queue", 5)  # pending messages; 0 = unbounded

        input_topic = self.get_parameter("input_topic").value
        self.model = self.get_parameter("model").value
        self.mode = self.get_parameter("mode").value
        self.output_dir = self.get_parameter("output_dir").value
        api_key_env = self.get_parameter("api_key_env").value
        api_key = os.getenv(api_key_env)

        if not api_key:
            raise RuntimeError(f"{api_key_env} is not set in the environment")
        if self.mode not in PLAYERS:
            raise RuntimeError(f"Invalid mode '{self.mode}' (use {', '.join(PLAYERS)})")
        player = PLAYERS[self.mode]
        if player and not shutil.which(player):
            raise RuntimeError(f"mode '{self.mode}' needs '{player}' on the PATH")

        # ---- ElevenLabs client ----
        self.client = ElevenLabs(api_key=api_key)
        self.voice_id = self._resolve_voice(self.get_parameter("voice").value)

        # ---- ROS interfaces ----
        # Transient-local so a mic node that starts mid-utterance still sees
        # the current speaking state.
        speaking_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.speaking_pub = self.create_publisher(Bool, "tts_speaking", speaking_qos)
        self._set_speaking(False)
        self.create_subscription(String, input_topic, self._on_text, 10)

        # ---- Worker thread: TTS calls block, so keep them out of the callback ----
        self._queue: "queue.Queue[str | None]" = queue.Queue(
            maxsize=self.get_parameter("max_queue").value
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"Listening on '{input_topic}' | voice_id={self.voice_id} | "
            f"model={self.model} | mode={self.mode}"
        )

    # ------------------------------------------------------------------ helpers
    def _resolve_voice(self, voice: str) -> str:
        """Accept a voice name on the account or a raw voice_id; fail otherwise."""
        try:
            for v in self.client.voices.get_all().voices:
                if v.name == voice:
                    return v.voice_id
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"Could not list ElevenLabs voices: {e}") from e
        try:
            return self.client.voices.get(voice).voice_id
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(
                f"'{voice}' is neither a voice name on this account nor a valid "
                f"voice_id ({e}). Add the voice to the account or pass its voice_id."
            ) from e

    def _set_speaking(self, value: bool):
        self.speaking_pub.publish(Bool(data=value))

    def _synthesize(self, text: str):
        return self.client.text_to_speech.convert(
            text=text,
            voice_id=self.voice_id,
            model_id=self.model,
            output_format="mp3_44100_128",
        )

    def _speak(self, text: str):
        audio = self._synthesize(text)  # iterator of bytes
        if self.mode == "stream":
            stream(audio)
        elif self.mode == "play":
            play(b"".join(audio))
        else:  # save
            os.makedirs(self.output_dir, exist_ok=True)
            name = hashlib.sha1(text.encode("utf-8")).hexdigest()[:16]
            path = os.path.join(self.output_dir, f"msg_{name}.mp3")
            save(b"".join(audio), path)
            self.get_logger().info(f"Saved audio to {path}")

    # ---------------------------------------------------------------- callbacks
    def _on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warn(f"TTS queue full; dropping: {text[:80]}")

    def _worker_loop(self):
        while True:
            text = self._queue.get()
            if text is None:
                return
            preview = text[:80] + ("..." if len(text) > 80 else "")
            self.get_logger().info(f"Speaking: {preview}")
            self._set_speaking(True)
            try:
                self._speak(text)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"TTS failed: {e}")
            finally:
                self._set_speaking(False)

    def shutdown(self):
        # Drop pending messages so the sentinel isn't stuck behind them.
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        self._queue.put(None)
        self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElevenLabsTTSNode()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger("elevenlabs_tts").fatal(str(e))
        raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
