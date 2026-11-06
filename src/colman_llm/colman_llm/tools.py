"""Tool schemas and allow-lists for the things Claude may ask the arm to do.

The enum values are human names. The node maps them to frames, so Claude never sees
or sends a raw frame name.
"""

# Tag IDs 1-4 are the white cubes and 5-8 the blue cubes. Cube 1 of each colour is the
# base of that colour's stack and never moves. The other three are stacked on top.
STACKS = {
    "white": {"base": "tag_1", "cubes": ["tag_2", "tag_3", "tag_4"]},
    "blue": {"base": "tag_5", "cubes": ["tag_6", "tag_7", "tag_8"]},
}
MAX_STACK = 3  # cubes on top of a base

# pick_and_place is kept in the code but is not offered to Claude in the cube demo,
# so it cannot disturb the stack state. Placeholder tag names, not on the table.
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

STACK_CUBE = {
    "name": "stack_cube",
    "description": (
        "Pick up loose cubes of one colour and stack them on that colour's base cube. "
        "Each cube is picked and placed on the top of the stack in turn. Takes about "
        "a minute per cube and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "color": {"type": "string", "enum": list(STACKS)},
            "count": {"type": "integer", "enum": list(range(1, MAX_STACK + 1))},
        },
        "required": ["color", "count"],
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

TOOLS = [STACK_CUBE, STOP_ROBOT, RESUME_ROBOT]


def known_things_prompt():
    """Describe the allow-lists for the system prompt so the two never drift apart."""
    colors = " and ".join(f"{c} cube" for c in STACKS)
    return (
        f"Known cubes: {colors}. Each colour has a base cube, and the arm can stack "
        f"up to {MAX_STACK} more cubes of that colour on it.\n"
    )
