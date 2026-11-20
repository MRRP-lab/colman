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

# Every cube by its spoken name; "white cube 1" is the base of the white stack.
CUBE_NAMES = {
    f"{color} cube {number}": frame
    for color, stack in STACKS.items()
    for number, frame in enumerate([stack["base"], *stack["cubes"]], start=1)
}
BASE_FRAMES = {stack["base"] for stack in STACKS.values()}

# pick_and_place moves one loose cube (a base never moves) onto any cube.
OBJECT_FRAMES = {n: f for n, f in CUBE_NAMES.items() if f not in BASE_FRAMES}
DESTINATION_FRAMES = dict(CUBE_NAMES)

# Named arm poses go_to_pose may use, by spoken name. The values are the pose names in
# colman_moveit_config/srdf/arm.srdf.xacro.
POSES = {
    "home": "home",
    "compact": "Compact",
}

PICK_AND_PLACE = {
    "name": "pick_and_place",
    "description": (
        "Pick up one loose cube and place it on top of another cube. Cube 1 of each "
        "colour is a base and cannot be picked up. A cube with another on top, or a "
        f"stack already {MAX_STACK} high, cannot be built on. To stack several cubes "
        "of one colour use stack_cube instead. Takes about a minute and blocks other "
        "arm commands."
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
    for color, stack in STACKS.items():
        names = [
            n for n, f in CUBE_NAMES.items() if f in [stack["base"], *stack["cubes"]]
        ]
        lines.append(
            f"The {color} cubes are {', '.join(names)}; {names[0]} is the base."
        )
    lines.append(
        f"A base never moves. Any stack can be built up to {MAX_STACK} cubes on top of "
        "its first cube."
    )
    lines.append("Named poses for go_to_pose: " + ", ".join(POSES) + ".")
    return "\n".join(lines) + "\n"
