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
| Table end | x ≈ 0.6 | Far edge from the robot (tag_3 and tag_6 at x ≈ 0.61) |
| Table sides | y ≈ ±0.55 | From the corner tags below |

## Table corners

The four AprilTags mark the corners of the table. Each tag is on top of a 50 mm cube, and the readings are the top centre of that cube. Measured positions in `base_link` (metres). Spread is the variation across samples.

| Tag | x | y | z | Samples | Spread (mm) |
|-----|-------|--------|-------|---------|-------------|
| tag_1 | 0.112 | -0.546 | 0.041 | 145 | under 0.1 |
| tag_3 | 0.616 | 0.546 | 0.038 | 309 | up to 0.2 |
| tag_5 | 0.117 | 0.541 | 0.045 | 24 | up to 0.6 |
| tag_6 | 0.612 | -0.555 | 0.038 | 232 | up to 0.3 |

The table spans x ≈ 0.11 to 0.62 and y ≈ -0.55 to +0.55, with the table surface at z = 0. The tag z of about 0.04 m is the top of the cubes, not the table surface. The centre of the corners is x ≈ 0.36, so the nominal centre of (0.3, 0) is approximate.

## Known discrepancies (calibration)

- The cubes are 50 mm tall, so the cube tops should read z = 0.050. The measured z values are 0.038 to 0.045 (5 to 12 mm low). This may be a calibration offset in z, or the tags may sit slightly below the cube top. The cause has not been determined.
- The readings are cube centres, so the physical table edges may sit slightly beyond the x and y corner values.
- The centre of the four corners is x ≈ 0.36, not the nominal 0.3.

## Open items

- Confirm which side of the robot +y is on in the real setup (tag_3 and tag_5 are at +y).

Keep targets inside these bounds when choosing named object and destination locations, so the LLM design in [`plan.md`](plan.md) only ever maps names to reachable positions.
