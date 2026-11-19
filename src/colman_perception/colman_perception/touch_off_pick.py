"""
Finds marks drawn on the table in an overhead image, by clicking on them.

    touch_off_pick <capture stem> <pixels.yaml> [<annotated.png>]

The stem has <stem>.npz and <stem>_rgb.png (from capture_grid.py). Click each
mark once. The click is refined to the centre of the mark within a few pixels,
and the mark is numbered in click order. The image is scaled to fit the
screen, so the clicks are mapped back to full resolution.

    u or Backspace   drop the last mark
    Enter or q       save and quit

Writes pixels.yaml (read by touch_off_check --pixels) and an image with the
numbers drawn on it, so you can see which mark is which when you touch them
with touch_off_record.
"""

import sys

import cv2
import yaml

from colman_perception.touch_off_check import load_capture, refine_mark

MAX_WIDTH = 1600
WINDOW = "touch_off_pick"


def draw(image, marks):
    out = image.copy()
    for label, (x, y) in enumerate(marks):
        cv2.drawMarker(out, (round(x), round(y)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(
            out,
            str(label),
            (round(x) + 10, round(y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )
    return out


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    stem, out_path = sys.argv[1:3]
    annotated_path = sys.argv[3] if len(sys.argv) > 3 else None

    _, image = load_capture(stem)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    zoom = min(1.0, MAX_WIDTH / image.shape[1])
    marks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            marks.append(refine_mark(gray, x / zoom, y / zoom))

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)
    while True:
        shown = cv2.resize(draw(image, marks), None, fx=zoom, fy=zoom)
        cv2.imshow(WINDOW, shown)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, ord("q")):
            break
        if key in (8, ord("u")) and marks:
            marks.pop()
    cv2.destroyAllWindows()

    points = [
        {"label": i, "pixel": [round(x, 2), round(y, 2)]}
        for i, (x, y) in enumerate(marks)
    ]
    with open(out_path, "w") as f:
        yaml.safe_dump({"points": points}, f, sort_keys=False, default_flow_style=None)
    if annotated_path:
        cv2.imwrite(annotated_path, draw(image, marks))
    print(f"wrote {len(points)} marks to {out_path}")


if __name__ == "__main__":
    main()
