"""Tool schema for the one thing Claude can do: move the arm to a saved position."""

# Spoken name -> pose name in colman_moveit_config/config/colman.srdf.
POSITIONS = {
    "position one": "Look1",
    "position two": "Look2",
}

MOVE_ARM = {
    "name": "move_arm",
    "description": "Move the robot arm to one of two saved positions.",
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "position": {"type": "string", "enum": list(POSITIONS)},
        },
        "required": ["position"],
        "additionalProperties": False,
    },
}

TOOLS = [MOVE_ARM]
