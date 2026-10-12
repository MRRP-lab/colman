import random


def left_to_right(tags):
    def y(name):
        return tags[name][1]

    return sorted(tags, key=y)


def right_to_left(tags):
    return list(reversed(left_to_right(tags)))


def random_pattern(tags, seed=None):
    names = sorted(tags)
    random.Random(seed).shuffle(names)
    return names


PATTERNS = {
    "left_to_right": left_to_right,
    "right_to_left": right_to_left,
    "random": random_pattern,
}
