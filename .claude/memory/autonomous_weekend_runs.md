---
name: autonomous_weekend_runs
description: Gilad's conventions for running the autonomous-project-manager agent over weekends / overnights on drone-follow plans. Read at the start of any unsupervised multi-day dispatch.
type: reference
---

# Autonomous weekend / overnight runs — conventions

Gilad runs multi-day plan execution unsupervised via the `autonomous-project-manager` agent (`~/.claude/agents/autonomous-project-manager.md`). These are the standing conventions; deviations need explicit per-run permission.

## Workflow

- **Pattern:** subagent-driven-development with mandatory two-stage review per task (spec → quality). For trivially small tasks, a combined spec+quality review is acceptable but never zero-review.
- **Three-strike rule:** 3 consecutive REWORK verdicts on the same task → hard stop, append diagnosis, wait for user.
- **TDD where the plan calls for it.** Plans 1–7 of the hailo_tiling refactor used the TDD pattern (failing test first, then implementation). Implementer prompts should pin this when the spec says so.

## State file

- Path: `docs/superpowers/overnight-manager-state.md`.
- Append-only. Never rewrite history. The manager's tick entries land at the tail.
- Sections in order: `## Active work`, `## Conventions`, `## Status-review checklist`, `## Notes / blockers`, dated tick entries, and on hard-stop a final `## ATTENTION REQUIRED` block.
- On wake-up, the user reads the tail. Format wake-up summaries with the headings: "Plans done", "Plans in flight", "Cleanup needed", "Decisions you need to make".

## Cron schedule

- Hourly status-review tick at **minute :17** (off-clock, spreads API load vs other cron consumers).
- Use `CronCreate(durable=true, recurring=true, cron="17 * * * *", prompt="<verbatim status-review checklist from state file>")`.
- On session-limit hit, schedule a one-shot resumption cron ~10 minutes past reset.

## Exclusive resources

- **One Hailo chip on this dev box.** Chip-using subagents (anything that runs a real HEF inference on the device) are strictly serial — only one in flight at a time. The state file tracks `chip_in_flight: <agent-id-or-null>`.
- Plans annotate chip-needing tasks with `[CHIP]`; pure-Python / MockBackend / doc / review work is `[no-chip]` and parallelizes freely.

## Out of scope for unsupervised runs

These plans need user attention and are NOT dispatched unsupervised:

- **Plan 6** — drone-follow migration. Touches `drone_follow/pipeline_adapter/` runtime hot path; regressions risk real flight behavior.
- **Plan 8** — submodule integration. May require patching `hailo-apps/` submodule; submodule push needs explicit user approval.
- **Plan 9** — paper / publication artifacts. Subjective writing, not algorithmic — needs human review per artifact.

If the caller's prompt explicitly authorizes one of these, fine — but the default is to defer.

## Verification — always use the project venv

The drone-follow project venv lives at `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/`. Every implementer prompt MUST include the binary's absolute path:

```
/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python -m pytest -q
```

Never tell implementers to run bare `pytest` or to rely on `source setup_env.sh` persisting — shell state is reset between Bash tool calls, and a bare `pytest` resolves to a different binary against a different Python.

The manager re-verifies test counts itself at the start of every cron tick, with the same binary.

## Git hygiene for parallel subagents

- Subagents share the working tree and `.git/index`. Multiple agents staging concurrently → races.
- Every implementer prompt MUST instruct: commit via a SINGLE Bash invocation `git add <explicit-files> && git commit -m "..."`. Never `git add .` or `git add -A`. Never split staging from commit across multiple Bash calls.
- File-scope-overlapping implementers run sequentially, not in parallel.
- Never push from the manager. Never rewrite shared history without explicit user authorization. Cleanup of mislabeled commits is a Monday task.

## Roll tuned defaults before user-facing release

Before any drone-follow release that involves controller params, sync `ControllerConfig` + `df_params.json` defaults from the air-unit's tuned `df_config.json`. Cross-ref: `feedback_roll_tuned_defaults.md`.
