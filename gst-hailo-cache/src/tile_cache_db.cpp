// gst-hailo-cache — C++ tile-cache DB helper (impl).
//
// Plan 5, Task 2. See tile_cache_db.hpp for the contract.
//
// Schema is taken LINE-FOR-LINE from hailo_tiling/cache/schema.sql; the
// Python SqliteCacheStore.open() uses con.executescript on that file, so
// any change here MUST be reflected there and vice versa.
//
// References to "Python side" below mean hailo_tiling/cache/store.py.

#include "tile_cache_db.hpp"

#include <sqlite3.h>

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace hailo_cache {

namespace {

// CREATE statements — identical to hailo_tiling/cache/schema.sql, just
// without the PRAGMA lines (we apply those programmatically).
constexpr const char* kCreateDetections =
    "CREATE TABLE IF NOT EXISTS detections ("
    "    frame_idx    INTEGER NOT NULL,"
    "    crop_x       INTEGER NOT NULL,"
    "    crop_y       INTEGER NOT NULL,"
    "    crop_w       INTEGER NOT NULL,"
    "    crop_h       INTEGER NOT NULL,"
    "    ppv          INTEGER NOT NULL,"
    "    dets_json    TEXT    NOT NULL,"
    "    ts_epoch     REAL    NOT NULL,"
    "    PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, ppv)"
    ") WITHOUT ROWID;";

constexpr const char* kCreateMeta =
    "CREATE TABLE IF NOT EXISTS meta ("
    "    k TEXT PRIMARY KEY,"
    "    v TEXT NOT NULL"
    ");";

constexpr const char* kInsertDetection =
    "INSERT INTO detections "
    "(frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json, ts_epoch) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?)";

constexpr const char* kSelectByKey =
    "SELECT frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json, ts_epoch "
    "FROM detections WHERE "
    "frame_idx=? AND crop_x=? AND crop_y=? AND crop_w=? AND crop_h=? AND ppv=?";

constexpr const char* kMetaSelect =
    "SELECT v FROM meta WHERE k = ?";

constexpr const char* kMetaUpsert =
    "INSERT INTO meta (k, v) VALUES (?, ?) "
    "ON CONFLICT(k) DO UPDATE SET v = excluded.v";

bool file_exists_(const std::string& path) {
    // Use sqlite3-friendly fopen("r"); avoids pulling in <filesystem>
    // dependency at link time (some toolchains require -lstdc++fs).
    if (FILE* f = std::fopen(path.c_str(), "rb")) {
        std::fclose(f);
        return true;
    }
    return false;
}

}  // namespace

// -- ctor/dtor/move ---------------------------------------------------------

TileCacheDb::~TileCacheDb() { close(); }

TileCacheDb::TileCacheDb(TileCacheDb&& other) noexcept
    : con_(other.con_), path_(std::move(other.path_)) {
    other.con_ = nullptr;
}

TileCacheDb& TileCacheDb::operator=(TileCacheDb&& other) noexcept {
    if (this != &other) {
        close();
        con_       = other.con_;
        path_      = std::move(other.path_);
        other.con_ = nullptr;
    }
    return *this;
}

// -- helpers ----------------------------------------------------------------

std::runtime_error TileCacheDb::make_error_(sqlite3* con, const char* what) {
    std::ostringstream oss;
    oss << "TileCacheDb: " << what;
    if (con) {
        const char* msg = sqlite3_errmsg(con);
        if (msg && *msg) {
            oss << ": " << msg;
        }
    }
    return std::runtime_error(oss.str());
}

void TileCacheDb::exec_(const char* sql) {
    char* err = nullptr;
    int rc = sqlite3_exec(con_, sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        std::ostringstream oss;
        oss << "TileCacheDb: exec failed: " << sql;
        if (err) {
            oss << " — " << err;
            sqlite3_free(err);
        }
        throw std::runtime_error(oss.str());
    }
}

void TileCacheDb::apply_schema_() {
    exec_(kCreateDetections);
    exec_(kCreateMeta);
}

void TileCacheDb::check_user_version_(bool was_new) {
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, "PRAGMA user_version", -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        throw make_error_(con_, "prepare PRAGMA user_version failed");
    }
    int uv = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        uv = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);

    if (was_new) {
        // Brand-new DB — explicitly stamp the schema version.
        // (sqlite3_exec rejects PRAGMA with bound params, so use a literal.)
        std::ostringstream oss;
        oss << "PRAGMA user_version = " << kSchemaVersion;
        exec_(oss.str().c_str());
        return;
    }

    if (uv == 0) {
        // Existed but unstamped (matches Python: apply schema then stamp).
        apply_schema_();
        std::ostringstream oss;
        oss << "PRAGMA user_version = " << kSchemaVersion;
        exec_(oss.str().c_str());
        return;
    }

    if (uv != kSchemaVersion) {
        std::ostringstream oss;
        oss << "TileCacheDb: " << path_ << ": cache schema_version mismatch "
            << "(file=" << uv << ", expected=" << kSchemaVersion << "). "
            << "Delete the file or use a matching hailo_tiling version.";
        // We already opened the connection; close it before throwing so
        // the caller doesn't see an "open" object in an error state.
        sqlite3_close(con_);
        con_ = nullptr;
        path_.clear();
        throw std::runtime_error(oss.str());
    }
}

// -- open/close -------------------------------------------------------------

void TileCacheDb::open(const std::string& path, bool create_if_missing) {
    if (con_) {
        throw std::runtime_error("TileCacheDb::open: already open");
    }
    const bool existed = file_exists_(path);
    if (!existed && !create_if_missing) {
        std::ostringstream oss;
        oss << "TileCacheDb::open: file does not exist and create_if_missing=false: "
            << path;
        throw std::runtime_error(oss.str());
    }

    int flags = SQLITE_OPEN_READWRITE;
    if (create_if_missing) {
        flags |= SQLITE_OPEN_CREATE;
    }
    sqlite3* con = nullptr;
    int rc = sqlite3_open_v2(path.c_str(), &con, flags, nullptr);
    if (rc != SQLITE_OK) {
        std::string msg = con ? sqlite3_errmsg(con) : "open failed";
        if (con) sqlite3_close(con);
        std::ostringstream oss;
        oss << "TileCacheDb::open: sqlite3_open_v2 failed for " << path
            << ": " << msg;
        throw std::runtime_error(oss.str());
    }
    con_  = con;
    path_ = path;

    // Pragmas — same order/values as Python SqliteCacheStore.open.
    try {
        exec_("PRAGMA journal_mode = WAL");
        exec_("PRAGMA synchronous = NORMAL");
        // Order matches Python: existing file → check user_version,
        // apply schema iff uv==0; new file → apply schema then stamp.
        if (existed) {
            // Don't blindly apply schema yet; check_user_version_
            // decides whether to apply (uv==0) or throw (uv mismatch)
            // or accept (uv==kSchemaVersion).
            check_user_version_(/*was_new=*/false);
        } else {
            apply_schema_();
            check_user_version_(/*was_new=*/true);
        }
    } catch (...) {
        // Roll back the open on any pragma/schema error.
        if (con_) {
            sqlite3_close(con_);
            con_ = nullptr;
        }
        path_.clear();
        throw;
    }
}

void TileCacheDb::close() {
    if (con_) {
        // Best-effort: ignore close errors.
        sqlite3_close(con_);
        con_ = nullptr;
    }
    path_.clear();
}

// -- meta -------------------------------------------------------------------

std::optional<std::string> TileCacheDb::meta_get(const std::string& key) {
    if (!con_) throw std::runtime_error("TileCacheDb::meta_get: not open");
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, kMetaSelect, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        throw make_error_(con_, "meta_get: prepare failed");
    }
    sqlite3_bind_text(stmt, 1, key.c_str(), -1, SQLITE_TRANSIENT);
    std::optional<std::string> out;
    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        const unsigned char* v = sqlite3_column_text(stmt, 0);
        out = v ? std::string(reinterpret_cast<const char*>(v)) : std::string{};
    } else if (rc != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        throw make_error_(con_, "meta_get: step failed");
    }
    sqlite3_finalize(stmt);
    return out;
}

void TileCacheDb::meta_put(const std::string& key, const std::string& value) {
    if (!con_) throw std::runtime_error("TileCacheDb::meta_put: not open");
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, kMetaUpsert, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        throw make_error_(con_, "meta_put: prepare failed");
    }
    sqlite3_bind_text(stmt, 1, key.c_str(),   -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, value.c_str(), -1, SQLITE_TRANSIENT);
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        throw make_error_(con_, "meta_put: step failed");
    }
}

// -- put_many ---------------------------------------------------------------

void TileCacheDb::put_many(const std::vector<Row>& rows) {
    if (!con_) throw std::runtime_error("TileCacheDb::put_many: not open");
    if (rows.empty()) return;

    exec_("BEGIN");
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, kInsertDetection, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        auto err = make_error_(con_, "put_many: prepare INSERT failed");
        // Best-effort rollback; ignore secondary error.
        sqlite3_exec(con_, "ROLLBACK", nullptr, nullptr, nullptr);
        throw err;
    }

    try {
        for (const auto& r : rows) {
            sqlite3_reset(stmt);
            sqlite3_clear_bindings(stmt);
            sqlite3_bind_int64(stmt, 1, r.frame_idx);
            sqlite3_bind_int  (stmt, 2, r.crop_x);
            sqlite3_bind_int  (stmt, 3, r.crop_y);
            sqlite3_bind_int  (stmt, 4, r.crop_w);
            sqlite3_bind_int  (stmt, 5, r.crop_h);
            sqlite3_bind_int  (stmt, 6, r.ppv);
            sqlite3_bind_text (stmt, 7, r.dets_json.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_double(stmt, 8, r.ts_epoch);
            int sr = sqlite3_step(stmt);
            if (sr != SQLITE_DONE) {
                throw make_error_(con_, "put_many: INSERT step failed");
            }
        }
    } catch (...) {
        sqlite3_finalize(stmt);
        sqlite3_exec(con_, "ROLLBACK", nullptr, nullptr, nullptr);
        throw;
    }
    sqlite3_finalize(stmt);
    exec_("COMMIT");
}

// -- get / get_many ---------------------------------------------------------

static std::optional<Row> read_one_row_(sqlite3_stmt* stmt) {
    Row r;
    r.frame_idx = sqlite3_column_int64(stmt, 0);
    r.crop_x    = sqlite3_column_int  (stmt, 1);
    r.crop_y    = sqlite3_column_int  (stmt, 2);
    r.crop_w    = sqlite3_column_int  (stmt, 3);
    r.crop_h    = sqlite3_column_int  (stmt, 4);
    r.ppv       = sqlite3_column_int  (stmt, 5);
    const unsigned char* j = sqlite3_column_text(stmt, 6);
    r.dets_json = j ? reinterpret_cast<const char*>(j) : "";
    r.ts_epoch  = sqlite3_column_double(stmt, 7);
    return r;
}

std::optional<Row> TileCacheDb::get(std::int64_t frame_idx,
                                    std::int32_t crop_x,
                                    std::int32_t crop_y,
                                    std::int32_t crop_w,
                                    std::int32_t crop_h,
                                    std::int32_t ppv) {
    if (!con_) throw std::runtime_error("TileCacheDb::get: not open");
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, kSelectByKey, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        throw make_error_(con_, "get: prepare failed");
    }
    sqlite3_bind_int64(stmt, 1, frame_idx);
    sqlite3_bind_int  (stmt, 2, crop_x);
    sqlite3_bind_int  (stmt, 3, crop_y);
    sqlite3_bind_int  (stmt, 4, crop_w);
    sqlite3_bind_int  (stmt, 5, crop_h);
    sqlite3_bind_int  (stmt, 6, ppv);
    rc = sqlite3_step(stmt);
    std::optional<Row> out;
    if (rc == SQLITE_ROW) {
        out = read_one_row_(stmt);
    } else if (rc != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        throw make_error_(con_, "get: step failed");
    }
    sqlite3_finalize(stmt);
    return out;
}

std::vector<std::optional<Row>>
TileCacheDb::get_many(std::int64_t frame_idx,
                      const std::vector<CropKey>& crops,
                      std::int32_t ppv) {
    if (!con_) throw std::runtime_error("TileCacheDb::get_many: not open");
    std::vector<std::optional<Row>> out;
    out.reserve(crops.size());

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(con_, kSelectByKey, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        throw make_error_(con_, "get_many: prepare failed");
    }

    try {
        for (const auto& c : crops) {
            sqlite3_reset(stmt);
            sqlite3_clear_bindings(stmt);
            sqlite3_bind_int64(stmt, 1, frame_idx);
            sqlite3_bind_int  (stmt, 2, c.x);
            sqlite3_bind_int  (stmt, 3, c.y);
            sqlite3_bind_int  (stmt, 4, c.w);
            sqlite3_bind_int  (stmt, 5, c.h);
            sqlite3_bind_int  (stmt, 6, ppv);
            int sr = sqlite3_step(stmt);
            if (sr == SQLITE_ROW) {
                out.push_back(read_one_row_(stmt));
            } else if (sr == SQLITE_DONE) {
                out.emplace_back();  // nullopt
            } else {
                throw make_error_(con_, "get_many: step failed");
            }
        }
    } catch (...) {
        sqlite3_finalize(stmt);
        throw;
    }
    sqlite3_finalize(stmt);
    return out;
}

}  // namespace hailo_cache
