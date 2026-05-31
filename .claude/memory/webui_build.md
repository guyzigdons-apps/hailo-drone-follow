---
name: webui_build
description: Web UI build requires Node ≥20.19 (Vite 8). System default on this dev box is Node 18 — must `nvm use 20` first. After build, output at robot_follow/ui/build/ and Python web server picks it up automatically.
type: reference
---

# Web UI build — Node version + workflow

## The trap
`robot_follow/ui/package.json` pulled in **Vite 8** as part of a security-vuln rebuild (`c48a65e1 fix ui vulnerabilities`). Vite 8 requires Node `^20.19.0 || >=22.12.0`. The dev box's nvm default is Node 18, so a naive `npm run build` fails with:
```
ReferenceError: CustomEvent is not defined
```
(Vite's CLI uses `CustomEvent` which only landed in Node 19+.)

## Build it
```bash
cd <repo-root>/robot_follow/ui
nvm use 20         # one-time per shell. nvm install 20 first if not present.
node --version     # should print v20.x
npm install
npm run build
```

Output: `<repo-root>/robot_follow/ui/build/`
- `index.html`
- `assets/index-<hash>.css`
- `assets/index-<hash>.js`

The Python web UI server reads from this `build/` directory at request time. **No restart needed** between rebuilds — refresh the browser (force-refresh to bust the cached hashed asset filenames).

## When to rebuild
- `package.json` changed (deps update / vuln fix)
- `package-lock.json` changed
- Any file under `robot_follow/ui/src/` changed (e.g. `App.jsx`)

`git diff --stat <prev>..<now> -- 'robot_follow/ui/'` to spot.

## Persistent fix
`nvm alias default 20` makes Node 20 the default for new shells. Currently the box defaults to 18 because other tooling expected it.
