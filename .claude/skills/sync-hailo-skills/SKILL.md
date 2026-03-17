---
name: sync-hailo-skills
description: "Sync skills and tools from hailo-apps submodule into this repo's .claude/skills/ via symlinks. Lists available skills, shows sync status, and creates/removes symlinks."
user-invocable: true
allowed-tools: Bash(ls *), Bash(ln *), Bash(rm *), Bash(readlink *), Bash(find *), Read, Glob
---

# Sync Hailo Skills

Manage symlinks from `hailo-apps/.claude/skills/` into this repo's `.claude/skills/`.

## Steps

1. **Discover** available skills in `hailo-apps/.claude/skills/` (each subdirectory with a SKILL.md)
2. **Check current state** of `.claude/skills/` — identify which are symlinks to hailo-apps, which are local skills, and which hailo-apps skills are missing
3. **Show status table** to the user:
   - ✓ linked — symlink exists and points to correct target
   - ✗ missing — available in hailo-apps but not linked
   - ⚠ broken — symlink exists but target missing
   - local — real directory, not a symlink (this repo's own skill)
4. **Ask user** which missing skills to link (or offer "link all")
5. **Create symlinks** using relative paths: `../../hailo-apps/.claude/skills/<name>`
6. **Report** final state

## Rules

- Never overwrite a local (non-symlink) skill directory
- Use relative symlinks so they work across clones
- The `sync-hailo-skills` skill itself is always local, never a symlink
- Symlink target path: `../../hailo-apps/.claude/skills/<skill-name>` (relative from `.claude/skills/`)
