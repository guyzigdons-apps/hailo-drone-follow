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

-- Generic key/value sidecar. Values are ALWAYS stored as TEXT; numeric
-- envelope keys are the decimal string of the integer (no padding).
--
-- Canonical provenance-envelope keys (shared spec for the C++ writer and the
-- Python reader — keep this list in sync with TileCacheDb / SqliteCacheStore):
--   video_w        source frame width in pixels (e.g. "3840")
--   video_h        source frame height in pixels (e.g. "2160")
--   resize_mode    how the source was mapped to the network input:
--                    "stretch"   — anisotropic scale, no padding
--                    "letterbox" — aspect-preserving scale + symmetric pad
--   dst_w          network input width in pixels (e.g. "640")
--   dst_h          network input height in pixels (e.g. "480")
--   interpolation  resize filter used; currently "linear"
--   hef_sha        hex SHA of the HEF that produced these detections
-- Any of these keys may be absent in older / partial caches; readers must
-- treat a missing key the same as meta_get() -> nullopt.
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

-- ReID embedding cache (Block R). One row per (frame_idx, crop_rect, model);
-- `vec` is a raw float32 little-endian blob (np.ndarray.tobytes()). Purely
-- additive — CREATE TABLE IF NOT EXISTS upgrades existing tile-cache DBs in
-- place on the next open() (no migration, no schema-version bump).
CREATE TABLE IF NOT EXISTS embeddings (
    frame_idx INTEGER NOT NULL,
    crop_x    INTEGER NOT NULL,
    crop_y    INTEGER NOT NULL,
    crop_w    INTEGER NOT NULL,
    crop_h    INTEGER NOT NULL,
    model     TEXT    NOT NULL,
    vec       BLOB    NOT NULL,
    ts_epoch  REAL    NOT NULL,
    PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, model)
) WITHOUT ROWID;
