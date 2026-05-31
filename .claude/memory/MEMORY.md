# Drone-Follow Memory Index

Sub-project knowledge for `community/apps/hailo_drone_follow`. Read at task start;
update when learning. Repo-wide knowledge lives in `.hailo/memory/`.

## Memories
- [hailotilecropper_dynamic](hailotilecropper_dynamic.md) — Community cropper plugin property surface (grid + dynamic + static), build/install path, why apps no longer hand-build grid strings.
- [webui_build](webui_build.md) — Vite 8 needs Node ≥20.19 but system default is Node 18. Use nvm before `npm install`/`npm run build`.
- [openhd_pairing](openhd_pairing.md) — `/usr/local/share/openhd/txrx.key` is authoritative. NEVER use `openhd --clean-start` on a paired link.
- [tracking_callback_risks](tracking_callback_risks.md) — Known-fragile spots in the detection callback + `highlight_target` pad probe (shared-ROI on tee'd buffer).
- [autonomous_weekend_runs](autonomous_weekend_runs.md) — Conventions for the `autonomous-project-manager` agent (state file, cron :17, chip serialization, venv path, parallel-agent git rules, Plans 6/8/9 out-of-scope).
- [gst_cache_source_pixel](gst_cache_source_pixel.md) — gst-hailo-cache provenance model: source-pixel crop keys, OpenCV resize contract (`resize-mode`, letterbox back-map bug), per-tile live-vs-cached bit-exact gate, detection serialization + recorder placement.

## Related (other folders)
- Sub-project entry: `../../CLAUDE.md` (architecture, CLI flags, OpenHD modes, sim, networking).
- ReID/tracking algorithm doc: `../../docs/tracking-reid-algorithm.md`.
- Skills (this sub-project): `../skills/drone-follow-dev/`, `../skills/safe-pull-and-rollback/`, `../skills/handling-anthropic-session-limits/`.
- Global agent: `~/.claude/agents/autonomous-project-manager.md` — dispatches the weekend run.
