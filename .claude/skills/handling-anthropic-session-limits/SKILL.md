---
name: handling-anthropic-session-limits
description: Detect, salvage, and recover from Anthropic per-account session-limit terminations during autonomous multi-agent runs. Use when a subagent returns with "session limit · resets HH:MM(am|pm) (TZ)" or when a long-running manager agent suspects its account budget has been exhausted.
---

# Handling Anthropic session limits

Anthropic enforces a per-account session budget. When it's hit, all in-flight subagents on that account terminate mid-task with an identical message. This skill is the detection + salvage + resume playbook, distilled from a real weekend run that hit the limit on Sunday morning.

## How a session limit manifests

When a subagent's session budget is exhausted, the `Agent` tool's return content ends with literally:

```
You've hit your session limit · resets HH:MM(am|pm) (TZ)
```

Notes:
- `HH:MM(am|pm)` is in the user's locale (e.g. `11:40am`).
- `(TZ)` is a TZ identifier (e.g. `(Asia/Jerusalem)`).
- The message arrives as a normal completion notification, NOT an exception. The Bash tool, Read tool, etc. all return success — only `Agent` returns show this string.
- Limits are **per-account**, not per-session. If one subagent hits the wall, every concurrently-running subagent on that account is likely about to terminate too. The next `Agent` dispatch will also fail-fast.

## Detection

After every `Agent` call, scan the return content with a substring match — don't over-fit a regex:

```python
text = agent_return_content
if "session limit" in text and "resets" in text:
    # session limit branch
    ...
```

Pull the reset time with a simple regex:

```python
import re
m = re.search(r"resets\s+(\d{1,2}:\d{2})(am|pm)\s+\(([^)]+)\)", text)
if m:
    local_time, ampm, tz = m.group(1), m.group(2), m.group(3)
```

Compute the reset as a UTC instant using `zoneinfo`:

```python
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo
h, mn = map(int, local_time.split(":"))
if ampm == "pm" and h != 12: h += 12
if ampm == "am" and h == 12: h = 0
now_local = datetime.now(ZoneInfo(tz))
reset_local = now_local.replace(hour=h, minute=mn, second=0, microsecond=0)
if reset_local <= now_local:
    reset_local += timedelta(days=1)   # already past today → tomorrow
reset_utc = reset_local.astimezone(ZoneInfo("UTC"))
```

## Salvage decision tree

When a subagent terminated mid-task, the working tree may carry partial work. Don't blindly commit or discard — triage:

```
START: subagent X returned with session-limit message
  │
  ├── git log shows X already committed before terminating?
  │     ├── YES → great, treat the task as DONE pending review; queue review for resume tick
  │     └── NO  → continue below
  │
  ├── git status: any modified or new files matching X's task scope?
  │     ├── NO files → X never reached the write step. Task is UNTOUCHED.
  │     │              Mark as needs-redispatch in state file, no cleanup needed.
  │     │
  │     ├── files present, look INCOMPLETE
  │     │   (heuristics: empty files, syntax errors visible in a Read,
  │     │    new test file with no test functions, header without
  │     │    matching .cpp, plan said N files but only N-2 exist)
  │     │   → DISCARD. `git restore --staged .` (if staged) and either
  │     │     `git restore <paths>` for tracked files or leave new
  │     │     untracked files in place (they cost nothing). Document.
  │     │
  │     └── files look COMPLETE
  │         (heuristics: all expected files present and non-empty,
  │          no obvious syntax errors on a quick Read, tests present
  │          alongside implementation, plan checklist appears satisfied)
  │         │
  │         ├── Can you run the task's verification cheaply?
  │         │   (pytest one file, gtest one binary, meson compile)
  │         │     ├── YES → run it. If passes → manager commits with
  │         │     │         message `<task title> [salvage from <agent-id>]`.
  │         │     │         Single Bash invocation: `git add <files> && git commit`.
  │         │     │         Do NOT add Co-Authored-By for an agent that
  │         │     │         never reported back cleanly.
  │         │     │
  │         │     └── NO  → leave in working tree. Document in state
  │         │              file as "needs human review-and-commit".
  │         │              Do NOT dispatch a "salvage agent" — that
  │         │              spends budget on something a human should look at.
  │         │
  │         └── (no cheap verification possible)
  │               → leave alone, document, defer to user.
```

Hint: a chip-using task (e.g. one that runs a real Hailo pipeline) often leaves NO observable working-tree changes — the artifacts land in `/tmp` or untracked output dirs. For those, salvage is essentially "unknown state, will re-dispatch on resume."

## Stop dispatching

Once the limit is detected:

1. **Don't try to dispatch more `Agent` calls in the same tick.** They'll fail-fast with the same message.
2. **Cheap tools (Bash, Read, Edit, Write) still work** — use them for triage, state-file updates, and salvage commits.
3. **Document the hit prominently in the state file.** Add a `## ATTENTION REQUIRED` or `## STOP DISPATCH — session limit hit` section near the tail.

## Schedule resumption

Use a one-shot durable cron job for ~10 minutes after the reset. The 10-minute buffer absorbs clock skew on Anthropic's side and gives the budget a moment to actually clear.

```python
# Convert reset_utc + 10 minutes into a cron spec
resume = reset_utc + timedelta(minutes=10)
cron_spec = f"{resume.minute} {resume.hour} {resume.day} {resume.month} *"
# e.g. "50 8 31 5 *" = May 31 at 08:50 UTC
```

Then (after loading the tool via `ToolSearch(query="select:CronCreate", max_results=1)`):

```
CronCreate(
  durable=true,
  recurring=false,
  cron=cron_spec,
  prompt="Session-limit reset reached. Read docs/superpowers/overnight-manager-state.md,
          verify budget restored by running a tiny no-op Agent dispatch, then resume
          dispatch per the queue documented in the state file's 'STOP DISPATCH' section."
)
```

When the cron fires, do a small canary `Agent` call (e.g. one quick read-only review) before resuming the full dispatch queue. If the canary still hits the limit, the reset estimate was wrong — re-schedule for another 30 minutes out.

## Document everything

The state-file entry for a session-limit hit must include:

```markdown
## YYYY-MM-DD HH:MM (UTC) — STOP DISPATCH, anthropic session limit reached

**Detection.** Subagents <id-1>, <id-2> returned with
"session limit · resets HH:MM(am|pm) (TZ)".

**State at termination.**
- HEAD = <SHA>
- Test suite: <count> passed + <skipped> skipped
- In-flight tasks: <list>
- Salvaged: <list with commit SHAs>
- Discarded: <list>
- Left in working tree for user review: <list>

**Resumption scheduled:** cron job <id> fires at <UTC instant> (= <local time>).

**What's blocked:** <list of dependent tasks that can't dispatch until resume>
```

This is what the user reads first when they wake up.

## Concrete example — weekend run 2026-05-31

State at detection:
- HEAD `cbf29d5`, Plan 7 fully done.
- 2 chip + chip-adjacent subagents in flight: Plan 5 Task 7 [CHIP] (`a6da4cf4919234243`) and Plan 5 Task 12 wrapper (`a62d17abd17d44b0c`).
- Both returned with the exact string `You've hit your session limit · resets 11:40am (Asia/Jerusalem)`.

Salvage outcome:
- Task 7 [CHIP]: no observable working-tree changes (chip-side artifacts live in untracked output dirs). Marked needs-redispatch on resume, no cleanup.
- Task 12 wrapper: 6 modified files + 3 new files matching the planned wrapper scope. Looked complete (header + impl + test + meson registration + plugin.cpp registration + README). No cheap verification available without rebuilding the plugin, which the manager declined to do because building the plugin is the next implementer's job, not the manager's. Documented as "needs human review-and-commit on wake-up." Manager did NOT commit on the user's behalf — the work was complete-looking but unverified, and the safer call was to defer.

Test suite confirmed at floor (235 + 3) after the manager's own pytest run. No regression. Dispatch stopped. Cron resumption scheduled for 11:50 Asia/Jerusalem (10 min past reset).

## Anti-pattern: trying to "push through"

Do NOT dispatch more `Agent` calls hoping one will get through. Every dispatch that hits the limit:
- Adds noise to the transcript.
- Wastes the small amount of tool-call budget the manager itself has.
- Confuses the next cron tick's triage (more "did this finish?" decisions).

Stop completely. Schedule. Document. Wait.

## Anti-pattern: committing salvaged WIP without verification

If you can't cheaply verify the salvaged work passes its own tests, **don't commit**. A bad commit on the branch costs more time to revert than a stale working tree costs to inspect. The default is "leave it, document it, let the user decide."

## Related skills

- `superpowers:subagent-driven-development` — the dispatch pattern this skill protects.
- `superpowers:verification-before-completion` — applies doubly when deciding whether to salvage.
- `~/.claude/agents/autonomous-project-manager.md` — the agent that uses this skill.
