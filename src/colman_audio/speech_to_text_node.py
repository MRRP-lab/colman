#!/usr/bin/env python3
"""ROS 2 node: Azure continuous speech recognition -> std_msgs/String topic.

Requires AZURE_TTS_KEY and AZURE_TTS_REGION in the environment.
Each finalized utterance is published immediately on the output topic.
"""
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import azure.cognitiveservices.speech as speechsdk


class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')

        self.declare_parameter('topic', 'speech_text')
        self.declare_parameter('language', 'en-US')

        key = os.getenv('AZURE_TTS_KEY')
        region = os.getenv('AZURE_TTS_REGION')
        if not key or not region:
            raise RuntimeError('Set AZURE_TTS_KEY and AZURE_TTS_REGION in your environment.')

        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(String, topic, 10)

        speech_config = speechsdk.SpeechConfig(subscription=key, region=region)
        speech_config.speech_recognition_language = self.get_parameter('language').value

        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.recognizer = speechsdk.SpeechRecognizer(
            speech_config=speech_config, audio_config=audio_config
        )

        # Azure fires these callbacks on its own thread; publishing from there is fine.
        self.recognizer.recognized.connect(self._on_recognized)
        self.recognizer.canceled.connect(self._on_canceled)
        self.recognizer.session_started.connect(
            lambda evt: self.get_logger().info('Speech session started'))
        self.recognizer.session_stopped.connect(
            lambda evt: self.get_logger().info('Speech session stopped'))

        self.recognizer.start_continuous_recognition_async().get()
        self.get_logger().info(f'Listening. Publishing recognized text on "{topic}".')

    def _on_recognized(self, evt):
        result = evt.result
        if result.reason == speechsdk.ResultReason.RecognizedSpeech and result.text:
            msg = String()
            msg.data = result.text
            self.pub.publish(msg)
            self.get_logger().info(f'Published: {result.text}')

    def _on_canceled(self, evt):
        details = evt.cancellation_details
        self.get_logger().error(f'Speech canceled: {details.reason}')
        if details.reason == speechsdk.CancellationReason.Error:
            self.get_logger().error(f'Error details: {details.error_details}')
            self.get_logger().error('Check AZURE_TTS_KEY, AZURE_TTS_REGION and your microphone.')

    def destroy_node(self):
        try:
            self.recognizer.stop_continuous_recognition_async().get()
        except Exception as exc:  # shutting down anyway
            self.get_logger().warn(f'Error stopping recognizer: {exc}')
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SpeechToTextNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
