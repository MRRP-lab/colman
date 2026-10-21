# Coordinate system

All Cartesian poses sent to the arm (`go_to_pose`, `MoveTo`, tag lookups) are expressed in the `base_link` frame, the base of the UR3e. Units are metres.

## Table layout (top view)

```text
                     +x  (away from the robot)
                      ^
                      |
         x = 0.6  ----+----  end of table
                      |
                      |
         x = 0.3      o      centre of table (0.3, 0)
                      |
                      |
  -y <----------------+----------------> +y
                    x = 0
                    (robot base)
```

## Reference points

| Point | Coordinates (x, y) | Notes |
|-------|--------------------|-------|
| Robot base | (0, 0) | Origin of `base_link` |
| Table centre | (0.3, 0) | |
| Table end | x ≈ 0.6 | Approximate; far edge from the robot |
| Table sides | y ≈ ±0.55 | From the AprilTag measurements below; the physical edge may sit slightly beyond the tags |

## AprilTag positions

Measured positions of the table tags in `base_link` (metres). Spread is the variation across samples.

| Tag | x | y | z | Samples | Spread (mm) |
|-----|-------|--------|-------|---------|-------------|
| tag_1 | 0.112 | -0.546 | 0.041 | 145 | under 0.1 |
| tag_3 | 0.616 | 0.546 | 0.038 | 309 | up to 0.2 |
| tag_5 | 0.117 | 0.541 | 0.045 | 24 | up to 0.6 |
| tag_6 | 0.612 | -0.555 | 0.038 | 232 | up to 0.3 |

The tags sit at the corners of a rectangle spanning x ≈ 0.11 to 0.62 and y ≈ -0.55 to +0.55. The tag z of about 0.04 m is roughly the table surface height. The centroid of the tags is x ≈ 0.36, slightly beyond the nominal table centre of 0.3, so the nominal centre is an approximation.

## Open items

- Confirm the physical table edges relative to the tags.
- Confirm which side of the robot +y is on in the real setup (tag_3 and tag_5 are at +y).

Keep targets inside these bounds when choosing named object and destination locations, so the LLM design in [`plan.md`](plan.md) only ever maps names to reachable positions.
