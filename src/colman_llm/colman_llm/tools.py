"""Tool schemas and allow-lists for the things Claude may ask the arm to do.

The enum values are human names. The node maps them to frames, so Claude never sees
or sends a raw frame name.
"""

# Placeholder tag names. Replace them when the real tags are named.
OBJECT_FRAMES = {
    "red block": "tag_3",
    "blue block": "tag_5",
}

DESTINATION_FRAMES = {
    "left tray": "tag_1",
    "right tray": "tag_6",
}

PICK_AND_PLACE = {
    "name": "pick_and_place",
    "description": (
        "Pick up one known object with the arm and place it at a known "
        "destination. Takes about a minute and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "object": {"type": "string", "enum": list(OBJECT_FRAMES)},
            "destination": {"type": "string", "enum": list(DESTINATION_FRAMES)},
        },
        "required": ["object", "destination"],
        "additionalProperties": False,
    },
}

STOP_ROBOT = {
    "name": "stop_robot",
    "description": "Stop the arm immediately. Use it whenever the user wants a stop.",
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {},
        "required": [],
        "additionalProperties": False,
    },
}

RESUME_ROBOT = {
    "name": "resume_robot",
    "description": (
        "Let the arm accept commands again after it was stopped. Only use it when the "
        "user clearly asks to resume."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {},
        "required": [],
        "additionalProperties": False,
    },
}

TOOLS = [PICK_AND_PLACE, STOP_ROBOT, RESUME_ROBOT]


def known_things_prompt():
    """Describe the allow-lists for the system prompt so the two never drift apart."""
    objects = ", ".join(OBJECT_FRAMES)
    destinations = ", ".join(DESTINATION_FRAMES)
    return f"Known objects: {objects}.\nKnown destinations: {destinations}.\n"
