# Implementation Plans Index

Tracks the decomposed plans derived from the master spec:
`docs/superpowers/specs/2026-05-28-tiling-library-design.md`.

| #   | Plan                                                  | Spec phases | Status     |
|-----|-------------------------------------------------------|-------------|------------|
| 1   | `2026-05-28-hailo-tiling-scaffold-and-scheduler-refactor.md` | 1, 2        | done       |
| 2   | Telemetry + new modifiers + backends ABC              | 3, 4, 5     | not started |
| 3   | FOV emulation source-data prep                        | 6           | not started |
| 4   | Cache schema + Python cache layer                     | 7           | not started |
| 5   | GStreamer cache plugins + hailo-apps-core patches     | 8, 14       | not started |
| 6   | GstCropperBackend + ablation harness                  | 9, 10       | not started |
| 7   | Telemetry import (ULG/SRT) + visualizer               | 11, 12      | not started |
| 8   | Drone-follow migration + RPI-GS data collection       | 13, 16      | not started |
| 9   | Paper-with-code artifacts                             | 15          | not started |
| (10)| DJI optical-zoom maximum-range bonus shoot            | 17          | bonus / ops |

Update the **Status** column as plans land. When a plan finishes, set its
status to `done` and bump the next plan to `in flight`.
