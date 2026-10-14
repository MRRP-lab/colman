#!/usr/bin/env python3
"""ROS 2 node: subscribes to LLM text output and speaks it with ElevenLabs TTS.

Subscribes:  <input_topic>  (std_msgs/String)  -- text from the LLM node
Publishes:   tts_speaking   (std_msgs/Bool)    -- True while audio is playing
                                                  (handy for muting the mic node)

Requires the ElevenLabs SDK >= 1.0:  pip install elevenlabs
Streaming/playing needs mpv (streaming) or ffmpeg (play) installed on the system.
"""

import hashlib
import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from elevenlabs import play, save, stream
from elevenlabs.client import ElevenLabs


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

        input_topic = self.get_parameter("input_topic").value
        self.model = self.get_parameter("model").value
        self.mode = self.get_parameter("mode").value
        self.output_dir = self.get_parameter("output_dir").value
        api_key = os.getenv(self.get_parameter("api_key_env").value)

        if not api_key:
            self.get_logger().fatal("ELEVENLABS_API_KEY is not set in the environment")
            raise SystemExit(1)
        if self.mode not in ("stream", "play", "save"):
            self.get_logger().fatal(f"Invalid mode '{self.mode}' (use stream, play, or save)")
            raise SystemExit(1)

        # ---- ElevenLabs client ----
        self.client = ElevenLabs(api_key=api_key)
        self.voice_id = self._resolve_voice(self.get_parameter("voice").value)

        # ---- ROS interfaces ----
        self.speaking_pub = self.create_publisher(Bool, "tts_speaking", 10)
        self.create_subscription(String, input_topic, self._on_text, 10)

        # ---- Worker thread: TTS calls block, so keep them out of the callback ----
        self._queue: "queue.Queue[str | None]" = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"Listening on '{input_topic}' | voice_id={self.voice_id} | "
            f"model={self.model} | mode={self.mode}"
        )

    # ------------------------------------------------------------------ helpers
    def _resolve_voice(self, voice: str) -> str:
        """Accept either a voice name (looked up on the account) or a raw voice_id."""
        try:
            for v in self.client.voices.get_all().voices:
                if v.name == voice:
                    return v.voice_id
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Could not list voices ({e}); treating '{voice}' as a voice_id")
            return voice
        self.get_logger().warn(f"No voice named '{voice}' found; treating it as a voice_id")
        return voice

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
        if text:
            self._queue.put(text)

    def _worker_loop(self):
        while True:
            text = self._queue.get()
            if text is None:
                return
            self.get_logger().info(f"Speaking: {text[:80]}{'...' if len(text) > 80 else ''}")
            self._set_speaking(True)
            try:
                self._speak(text)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"TTS failed: {e}")
            finally:
                self._set_speaking(False)

    def shutdown(self):
        self._queue.put(None)
        self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ElevenLabsTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()