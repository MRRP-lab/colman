"""Spoken stop detection. Runs in the subscription callback, before queueing."""

import re

# Whole words only. This accepts false matches such as "don't stop at the red
# block", which is the safe direction for a stop command.
_STOP = re.compile(r"\b(stop|halt|freeze)\b", re.IGNORECASE)


def is_stop_command(text):
    return bool(_STOP.search(text))
