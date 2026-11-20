"""Tool schemas and allow-lists for the things Claude may ask the arm to do.

The enum values are human names. The node maps them to frames, so Claude never sees
or sends a raw frame name.
"""

# Tag IDs 1-4 are the white cubes and 5-8 the blue cubes. No cube is special: any cube
# can be picked up (when nothing is on it) and placed on any other cube, of any colour,
# to any height the arm can reach. stack_cube only builds a stack of one colour.
STACKS = {
    "white": ["tag_1", "tag_2", "tag_3", "tag_4"],
    "blue": ["tag_5", "tag_6", "tag_7", "tag_8"],
}
# Most cubes stack_cube is asked to put in one stack (one colour has four cubes).
MAX_STACK = 4

# Every cube by its spoken name. The numbers only tell cubes of a colour apart.
CUBE_NAMES = {
    f"{color} cube {number}": frame
    for color, frames in STACKS.items()
    for number, frame in enumerate(frames, start=1)
}

# pick_and_place moves any cube that has nothing on it onto any other cube.
OBJECT_FRAMES = dict(CUBE_NAMES)
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
        "Pick up one cube and place it on top of another cube, of any colour. The "
        "cube must have nothing on it, and neither cube may be the same one. It can "
        "go on a stack of any height the arm can reach. To stack several cubes of one "
        "colour use stack_cube instead. Takes about a minute and blocks other arm "
        "commands."
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
        "Build one stack of cubes of one colour, in whatever order is quickest. count "
        "is how many cubes of that colour end up in the stack, counting the one that "
        "stays where it is. It starts from the tallest stack of that colour, or from "
        "one cube if there is none, and puts the loose cubes of that colour on it one "
        "at a time. Takes about a minute per cube and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "color": {"type": "string", "enum": list(STACKS)},
            "count": {"type": "integer", "enum": list(range(2, MAX_STACK + 1))},
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
    for color, frames in STACKS.items():
        names = [n for n, f in CUBE_NAMES.items() if f in frames]
        lines.append(f"The {color} cubes are {', '.join(names)}.")
    lines.append(
        "No cube is a base: any cube with nothing on it can go on any other cube, in "
        "any order and to any height. stack_cube builds one stack of a single colour "
        f"from two to {MAX_STACK} cubes."
    )
    lines.append("Named poses for go_to_pose: " + ", ".join(POSES) + ".")
    return "\n".join(lines) + "\n"
