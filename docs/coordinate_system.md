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
| Table sides | y extends in both -y and +y | Extent not yet measured |

## Open items

- Measure the table's half-width in y and record the limits here.
- Record the table surface height (z) in `base_link`.
- Confirm sign conventions of +y (left or right of the robot) against the real setup.

Keep targets inside these bounds when choosing named object and destination locations, so the LLM design in [`plan.md`](plan.md) only ever maps names to reachable positions.
