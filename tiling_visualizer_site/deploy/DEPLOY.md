# Hailo Dynamic Tiling Showcase — Self-Contained Package

This directory is a **fully self-contained static website**. It needs no Python
runtime, no Node.js, no Hailo stack, and no database to serve — only a static
file server. Everything required (HTML/CSS/JS, the transcoded 1080p video, and
the detection/tile data) lives inside this directory.

## What's inside

```
.
├── index.html              # the visualizer app
├── css/  js/               # vanilla JS/CSS (no build step)
├── data/
│   ├── manifest.json       # content index
│   ├── videos/
│   │   └── 0007_native.mp4 # 1080p H.264 transcode of the showcase clip
│   └── runs/
│       └── 0007_showcase.frames.json   # per-frame detections + tiles
├── serve.py                # zero-dependency local dev server (stdlib only)
└── deploy/
    └── nginx.conf          # production server block (recommended)
```

The video carries the tiles/detections as **data** (`frames.json`), not burned
into the pixels — the browser draws the overlay live on an HTML5 canvas, so you
can toggle the target/tiles, scrub frame-by-frame, and change the confidence
threshold. The locked target is drawn red; dynamic ROI tiles are green, the
dense search grid cyan (dashed), and pyramid acquisition tiles yellow (dashed).

## Caching model

- The MP4 is **one immutable file** served straight off disk — never
  re-encoded per request. The OS file cache keeps it hot in RAM.
- The **browser caches it on the user's machine**. With nginx (HTTP Range
  requests), seeking fetches only the needed byte ranges and caches them, so
  re-watching/seeking back does not re-download.
- `frames.json` is fetched once, parsed, and held in browser memory.

So the only server-side cost is disk + bandwidth; nothing is generated live.

---

## Option A — Quick local test (no install)

Requires Python 3.8+ only:

```bash
python3 serve.py --port 8123 --dir .
# → http://localhost:8123
```

> `serve.py` implements HTTP Range requests (`206 Partial Content`), so video
> seeking and frame-stepping work locally. (Plain `python -m http.server` does
> **not** — avoid it here.) For a shared/production deployment use nginx, which
> also handles Range and adds gzip + long-lived caching.

## Option B — Production on a Hailo server (nginx, recommended)

```bash
sudo apt-get install -y nginx
sudo cp -r .  /var/www/tiling-showcase
sudo cp deploy/nginx.conf /etc/nginx/sites-available/tiling-showcase
sudo ln -sf /etc/nginx/sites-available/tiling-showcase /etc/nginx/sites-enabled/
# edit server_name in the conf to match your host, then:
sudo nginx -t && sudo systemctl reload nginx
```

Browse to `http://<server>/`.

## Option C — rsync to an existing web root

```bash
rsync -av --delete ./  <server>:/var/www/tiling-showcase/
```

---

## Keyboard shortcuts (in the viewer)

`Space` play/pause · `[` `]` step one frame · `Home`/`End` first/last frame ·
`f` reset zoom · `+`/`-` zoom · `t` cycle tile source · `,`/`.` playback speed ·
`g` go-to-frame · `?` help overlay.
