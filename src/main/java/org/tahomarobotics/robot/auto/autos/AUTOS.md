> Differences per side are denoted with `Left/Right` (i.e. `1/2`), further differences by alliance are
> denoted with `(Blue: Left/Right)|(Red: Left/Right)` (i.e. `1/2|3/4`).

`SCORE_DISTANCE` = `1"`
`SCORING_TIME` = `0.25s`
`COLLECT_DURATION` = `1s`

### Without Coral Detection

1. Assume cameras are returning tolerable positions, and we properly
   have a piece preloaded, and the cameras are pointed towards our first scoring position.
    - If cameras are not working, `A-STOP`.
2. In sequence:
    1. In parallel:
        1. Given our auto starting state (with the elevator down, arm vertical,
           and collector back), zero the collector and elevator again the hard-stops.
        2. Drive to pole `J/E` via an approach point `18"` in front of the scoring position.
            1. When we are at the approach point, put the arm up to `L4` from `STOW`.
            2. When we are `SCORE_DISTANCE` from the scoring position, score the grabber for `SCORING_TIME`.
    2. In parallel:
        1. Pre-calculate the angle from the scoring position to the coral station
           collection waypoint. Using that angle, drive to the collection waypoint in a
           straight line. During the driving, isolate on `K/D`'s AprilTag.
        2. Move the arm to `STOW`, then to `COLLECT`.
    3. In parallel:
        1. Drive to pole `K/D` via an approach point `18"` in front of the scoring position.
            1. When we are at the approach point, if the grabber is collected, put the arm up to `L4` from `STOW`.
            2. When we are `SCORE_DISTANCE` from the scoring position, score the grabber for `SCORING_TIME`.
        2. When we are collected, put the arm to `STOW`.
            1. If we are not collected by `COLLECT_DURATION`, **cancel this command group**.
    4. Repeat `2.2` onwards for `L/C`, `A/B`, and `A/B` but `L3`
       (or shift all scoring positions back one, to make `I/F` the first scoring position).