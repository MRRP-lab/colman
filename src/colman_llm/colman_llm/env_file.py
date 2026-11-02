"""Minimal .env loader so ANTHROPIC_API_KEY can live in the repository's .env."""

import os
from pathlib import Path


def find_env_file():
    """Return the .env at $PIXI_PROJECT_ROOT, or the nearest one above the cwd."""
    root = os.getenv("PIXI_PROJECT_ROOT")
    if root:
        candidate = Path(root) / ".env"
        return candidate if candidate.is_file() else None
    cwd = Path.cwd()
    for folder in [cwd, *cwd.parents]:
        if (folder / ".env").is_file():
            return folder / ".env"
    return None


def parse_env_file(text):
    """Parse KEY=VALUE lines. Spaces around '=' and quotes around values are allowed."""
    values = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.removeprefix("export ").strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key] = value
    return values


def load_env_file():
    """Set variables from .env without overriding ones already in the environment."""
    path = find_env_file()
    if path is None:
        return
    for key, value in parse_env_file(path.read_text()).items():
        os.environ.setdefault(key, value)
