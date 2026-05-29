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

-- Full-frame results (spec §7.8 / §7.13). Written by hailocachewriter
-- mode=full_frame AFTER hailodetiler — one row per (frame_idx, ppv) holding
-- source-frame-coord aggregated detections plus the tile layout that was
-- used this frame.  This table is INDEPENDENT of `detections`: full_frame
-- mode and tile_cache mode are typically written to SEPARATE output files
-- (different `output-file=` properties on the writer), so a real-world DB
-- usually contains only ONE of the two tables.  Both are listed here so
-- `SqliteCacheStore.open` accepts files produced by either writer (or by
-- the rare combined-table file) without schema-version drift.
CREATE TABLE IF NOT EXISTS frame_results (
    frame_idx    INTEGER NOT NULL,
    ppv          INTEGER NOT NULL,
    dets_json    TEXT    NOT NULL,
    tiles_json   TEXT    NOT NULL,
    ts_epoch     REAL    NOT NULL,
    PRIMARY KEY (frame_idx, ppv)
) WITHOUT ROWID;
