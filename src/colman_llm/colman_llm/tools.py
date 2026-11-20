"""Tool schemas and allow-lists for the things Claude may ask the arm to do.

The enum values are human names. The node maps them to frames, so Claude never sees
or sends a raw frame name.
"""

# Tag IDs 1-4 are the white cubes and 5-8 the blue cubes. No cube is special: any cube
# can be moved, and a stack can hold any cubes in any order.
COLORS = {
    "white": ["tag_1", "tag_2", "tag_3", "tag_4"],
    "blue": ["tag_5", "tag_6", "tag_7", "tag_8"],
}
CUBES_PER_COLOR = 4
MIN_STACK = 2  # the smallest stack stack_cube can be asked for

# Every cube by its spoken name: "white cube 1" is tag_1 and "blue cube 4" is tag_8.
# The number only tells cubes of one colour apart. It does not rank them.
CUBE_NAMES = {
    f"{color} cube {number}": frame
    for color, frames in COLORS.items()
    for number, frame in enumerate(frames, start=1)
}

# Named arm poses go_to_pose may use, by spoken name. The values are the pose names in
# colman_moveit_config/srdf/arm.srdf.xacro.
POSES = {
    "home": "home",
    "compact": "Compact",
}

PICK_AND_PLACE = {
    "name": "pick_and_place",
    "description": (
        "Pick up one cube and place it on top of another cube. Either cube can be any "
        "colour and either can already be part of a stack, but the cube picked up must "
        "have nothing on top of it and so must the cube it goes on. To stack several "
        "cubes of one colour use stack_cube instead. Takes about a minute and blocks "
        "other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "object": {"type": "string", "enum": list(CUBE_NAMES)},
            "destination": {"type": "string", "enum": list(CUBE_NAMES)},
        },
        "required": ["object", "destination"],
        "additionalProperties": False,
    },
}

GO_TO_POSE = {
    "name": "go_to_pose",
    "description": (
        "Move the empty arm to a named pose: "
        + ", ".join(POSES)
        + ". 'home' stands the arm up with the forearm level, 'compact' folds it. "
        "It refuses while the arm is holding a cube or busy."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {"pose": {"type": "string", "enum": list(POSES)}},
        "required": ["pose"],
        "additionalProperties": False,
    },
}

STACK_CUBE = {
    "name": "stack_cube",
    "description": (
        "Build a stack of cubes of one colour. count is how many cubes the finished "
        "stack has. It builds on the tallest stack of that colour already on the "
        "table, or on any cube of that colour if there is none, and adds cubes of "
        "that colour one at a time in any order. No cube is the base. Takes about a "
        "minute per cube and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "color": {"type": "string", "enum": list(COLORS)},
            "count": {
                "type": "integer",
                "enum": list(range(MIN_STACK, CUBES_PER_COLOR + 1)),
            },
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

RELEASE_CUBE = {
    "name": "release_cube",
    "description": (
        "Let go of the cube the arm is holding, where it is, by turning the vacuum "
        "off. Only use it when the user asks to drop or release the cube, for example "
        "after a failed placement. It refuses while the arm is busy moving."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {},
        "required": [],
        "additionalProperties": False,
    },
}

REPORT_STATE = {
    "name": "report_state",
    "description": (
        "Report which cubes are stacked or loose, what the arm is holding, and whether "
        "it is stopped. It never moves the arm. Use it to answer questions about the "
        "table or the arm instead of answering from memory."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {},
        "required": [],
        "additionalProperties": False,
    },
}

TOOLS = [
    STACK_CUBE,
    PICK_AND_PLACE,
    GO_TO_POSE,
    RELEASE_CUBE,
    REPORT_STATE,
    STOP_ROBOT,
    RESUME_ROBOT,
]


def known_things_prompt():
    """Describe the allow-lists for the system prompt so the two never drift apart."""
    lines = []
    for color, frames in COLORS.items():
        names = [n for n, f in CUBE_NAMES.items() if f in frames]
        lines.append(f"The {color} cubes are {', '.join(names)}.")
    lines.append(
        "Any cube can be moved and any cube can go on any other, in any colour or "
        "order. No cube is a base. Only a cube with nothing on top of it can be moved "
        "or built on."
    )
    lines.append("Named poses for go_to_pose: " + ", ".join(POSES) + ".")
    return "\n".join(lines) + "\n"
