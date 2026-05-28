-- hailo_tiling/cache/schema.sql
--
-- Tile-cache schema. Same file Plan 5's libhailotile_cache.so will produce.
-- One row per (frame_idx, crop_rect, ppv); coordinates are source-pixel ints.

PRAGMA user_version = 1;
PRAGMA journal_mode = WAL;

CREATE TABLE IF NOT EXISTS detections (
    frame_idx    INTEGER NOT NULL,
    crop_x       INTEGER NOT NULL,
    crop_y       INTEGER NOT NULL,
    crop_w       INTEGER NOT NULL,
    crop_h       INTEGER NOT NULL,
    ppv          INTEGER NOT NULL,
    dets_json    TEXT    NOT NULL,
    ts_epoch     REAL    NOT NULL,
    PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, ppv)
) WITHOUT ROWID;

CREATE TABLE IF NOT EXISTS meta (
    k TEXT PRIMARY KEY,
    v TEXT NOT NULL
);
