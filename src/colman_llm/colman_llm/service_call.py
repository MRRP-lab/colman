"""Blocking call of one std_srvs/Trigger service from the worker thread.

The client is passed in, so tests can use a fake one without ROS.
"""

import threading

SERVICE_WAIT_S = 2.0


class ServiceCall:
    def __init__(self, client, request, name, logger):
        self._client = client
        self._request = request
        self._name = name
        self._log = logger

    def call(self, timeout_s=5.0):
        """Call the service and wait. Return (ok, message) in plain language."""
        if not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False, f"The /{self._name} service is not available."
        done = threading.Event()
        box = {}

        def on_done(future):
            box["response"] = future.result()
            done.set()

        self._client.call_async(self._request).add_done_callback(on_done)
        if not done.wait(timeout_s):
            return False, f"The /{self._name} service did not answer."
        response = box["response"]
        self._log.info(f"/{self._name}: {response.success} {response.message}")
        if not response.success:
            return False, response.message or f"/{self._name} failed."
        return True, response.message or f"/{self._name} succeeded."

    def call_async(self):
        """Fire the service call without waiting, for use in a subscription callback."""
        if not self._client.wait_for_service(timeout_sec=0.0):
            self._log.error(f"The /{self._name} service is not available.")
            return

        def on_done(future):
            response = future.result()
            self._log.info(f"/{self._name}: {response.success} {response.message}")

        self._client.call_async(self._request).add_done_callback(on_done)
