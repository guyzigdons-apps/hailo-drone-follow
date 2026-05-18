---
name: safe-pull-and-rollback
description: Tag-before-pull pattern with diff, rollback, and cross-machine sharing. Use when the user is about to pull from origin and wants a safety net, or wants to compare what's about to land, or wants to revert after a pull broke something.
---

# Safe pull / rollback / compare

The drone-follow branch (`hailo_drone_follow`) tends to drift far ahead of local checkouts (we've seen 28-commit-ahead pulls touching 23 files). Always tag before fast-forwarding so you can A/B test and roll back.

## Tag the current tip

```bash
TAG="pre-pull-hailo_drone_follow-$(date +%Y%m%d)"
git tag -a "$TAG" -m "Local hailo_drone_follow tip before pulling from origin"
git rev-parse "$TAG^{commit}"   # confirm it points at HEAD
```

Use an annotated tag (`-a`), not a lightweight tag — annotated tags carry a message, push cleanly, and survive `git gc` better.

## Inspect what's about to land

```bash
git fetch --quiet
git log HEAD..origin/hailo_drone_follow --oneline                # commit list
git diff --stat HEAD..origin/hailo_drone_follow | head -30       # file-level
git diff --stat HEAD..origin/hailo_drone_follow -- \
  community/apps/hailo_drone_follow/drone_follow/pipeline_adapter/   # tracking files only
```

For a deeper read on tracking-affecting changes, see `../../memory/tracking_callback_risks.md`.

## Pull

```bash
git pull --ff-only          # refuse if not fast-forwardable
```

Fast-forward only — if it can't, your local has commits the remote doesn't and you should rebase or merge deliberately.

## Diff against the saved tag (post-pull)

```bash
git diff <TAG>..HEAD                                    # full diff
git diff <TAG>..HEAD -- <path>                          # scoped
git log <TAG>..HEAD --oneline                           # commits since tag
```

## Roll back

```bash
git reset --hard <TAG>           # local branch rewinds to tag
# untracked files survive a hard reset; only tracked files are reset
```

Or, to look around without moving the branch:

```bash
git checkout <TAG>               # detached HEAD
# ...inspect, run...
git switch -                     # back to previous branch
```

## Push the tag (so other machines / the air unit can use it)

```bash
git push origin <TAG>
git ls-remote --tags origin <TAG>    # verify
```

On the other machine:

```bash
git fetch --tags origin
git checkout <TAG>                       # detached HEAD
git switch -c pre-pull-test <TAG>        # or a branch
```

## When you have uncommitted changes

A simple `git checkout <other-branch>` works **only if** the files you've modified have identical content on both branches. Otherwise:
- Tracked-file changes against a divergent base ⇒ stash first: `git stash --include-untracked`.
- Untracked files always travel with `checkout` — no stash needed for them.

## Don'ts

- Don't `git pull` (without `--ff-only`) when behind — it'll merge and create a noisy commit.
- Don't `git reset --hard` without first verifying the tag exists: `git rev-parse <TAG>`.
- Don't push tags to origin before confirming the user wants them shared. Tags pushed accidentally are awkward to remove (`git push origin :refs/tags/<TAG>` is the cleanup but takes a round-trip).
