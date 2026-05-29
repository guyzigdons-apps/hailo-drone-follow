// gst-hailo-cache — lookup latency + writer microbench.
//
// Plan 5 Task 10 — two sub-benches in one binary, wired as a meson
// benchmark() target (`meson test -C build --benchmark`):
//
//   Reader bench
//   ------------
//     10 000 `tile_cache_db::get` calls against a 10 000-row populated
//     DB (random keys, 50% hit). Report median / p95 / p99 latency in
//     microseconds. Spec §7.9 bar: < 1 ms / crop ⇒ p99 < 1000 µs.
//
//   Writer bench
//   ------------
//     10 000 `transform_ip` calls in a `hailocachewriter` element driven
//     by a real GStreamer pipeline (videotestsrc → writer → fakesink).
//     We measure streaming-thread time via two pad probes wrapping the
//     writer: BUFFER probe on the sink pad records t0, BUFFER probe on
//     the src pad records dt. Report median / p95 / p99 of dt. Spec
//     §7.8 bar: < 100 µs streaming-thread p99.
//
// Output: percentiles + PASS / FAIL relative to the bars are written
// to both stdout AND `tests/bench-results.txt` (relative to CWD when
// meson invokes the binary — meson sets CWD to the source dir of the
// tests/meson.build at run time; we resolve relative to the bench
// binary's directory to be deterministic).
//
// Exit code:
//   0 — both benches PASS
//   1 — at least one bench FAIL (p99 above bar)
//   2 — harness / pipeline error
//
// Notes:
//   * Reader bench uses TileCacheDb directly (no GStreamer in the hot
//     path). It is single-threaded.
//   * Writer bench drives the real plugin element via a parse-launched
//     pipeline, so it exercises ring-push + writer-thread plumbing
//     exactly as production does. The pad-probe diff captures the
//     streaming-thread cost only — the background writer thread does
//     SQLite COMMITs off-path and does NOT contribute to the latency
//     numbers reported here (it just needs to drain fast enough that
//     the ring doesn't fill up; we measure dropped-rows post-EOS and
//     warn if any were dropped).

#include "tile_cache_db.hpp"

#include <gst/gst.h>

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using clock_t_ = std::chrono::steady_clock;

// Spec bars in microseconds.
constexpr double kReaderBarUs = 1000.0;   // §7.9: < 1 ms / crop
constexpr double kWriterBarUs = 100.0;    // §7.8: < 100 µs streaming p99

constexpr int kReaderIter   = 10000;
constexpr int kReaderDbRows = 10000;
constexpr int kWriterIter   = 10000;
constexpr int kWarmupIter   = 1000;

struct PercentileResult {
    double median;
    double p95;
    double p99;
};

PercentileResult compute_percentiles(std::vector<double>& samples_us) {
    std::sort(samples_us.begin(), samples_us.end());
    const size_t n = samples_us.size();
    PercentileResult r{};
    if (n == 0) return r;
    auto pct = [&](double p) -> double {
        // Linear index pick — for monitoring purposes, no need for
        // fancy interpolation. Clamp to [0, n-1].
        size_t idx = (size_t)((p / 100.0) * (double)(n - 1) + 0.5);
        if (idx >= n) idx = n - 1;
        return samples_us[idx];
    };
    r.median = pct(50.0);
    r.p95    = pct(95.0);
    r.p99    = pct(99.0);
    return r;
}

void print_bench_result(std::ostream& os, const char* name,
                        const PercentileResult& r, double bar_us,
                        bool passed) {
    os << name << ":\n"
       << "  samples: 10000\n"
       << "  median = " << r.median << " us\n"
       << "  p95    = " << r.p95    << " us\n"
       << "  p99    = " << r.p99    << " us\n"
       << "  bar    = " << bar_us   << " us (p99 threshold)\n"
       << "  result = " << (passed ? "PASS" : "FAIL") << "\n\n";
}

// ---------------------------------------------------------------------------
// Reader bench.
// ---------------------------------------------------------------------------

bool run_reader_bench(std::ostream& report) {
    // Populate a 10 000-row SQLite via TileCacheDb directly.
    const std::string db_path = std::string("/tmp/bench_reader_") +
                                std::to_string(::getpid()) + ".sqlite3";
    ::unlink(db_path.c_str());
    ::unlink((db_path + "-wal").c_str());
    ::unlink((db_path + "-shm").c_str());

    hailo_cache::TileCacheDb db;
    try {
        db.open(db_path);
    } catch (const std::exception& e) {
        std::cerr << "reader bench: TileCacheDb::open failed: " << e.what() << "\n";
        return false;
    }

    // Generate kReaderDbRows unique (frame_idx, x, y, w, h, ppv) keys.
    // Use a deterministic seed so the bench is reproducible.
    std::mt19937 rng(0xC0FFEE);
    std::uniform_int_distribution<int> frame_dist(0, 999);     // 1000 frames
    std::uniform_int_distribution<int> coord_dist(0, 39);      // 40 coord buckets
    constexpr int kPpv = 1;
    constexpr int kCellW = 320;
    constexpr int kCellH = 240;

    struct Key {
        std::int64_t frame_idx;
        std::int32_t x, y, w, h;
    };
    std::vector<Key>             keys;
    std::vector<hailo_cache::Row> rows;
    keys.reserve(kReaderDbRows);
    rows.reserve(kReaderDbRows);

    // We need unique composite PKs. Build a deduped set by stepping
    // through a structured grid (frame_idx, x, y) until we hit the row
    // count. This gives a deterministic, gap-free key space we can also
    // sample miss keys from.
    {
        std::vector<Key> all;
        for (int f = 0; f < 200 && (int)all.size() < kReaderDbRows; ++f) {
            for (int xi = 0; xi < 40 && (int)all.size() < kReaderDbRows; ++xi) {
                for (int yi = 0; yi < 40 && (int)all.size() < kReaderDbRows; ++yi) {
                    Key k;
                    k.frame_idx = f;
                    k.x = xi * 32;
                    k.y = yi * 24;
                    k.w = kCellW;
                    k.h = kCellH;
                    all.push_back(k);
                }
            }
        }
        // Shuffle so insertion order isn't column-clustered (better
        // approximation of real-world write pattern).
        std::shuffle(all.begin(), all.end(), rng);
        keys = std::move(all);
    }

    rows.clear();
    rows.reserve(keys.size());
    for (const Key& k : keys) {
        hailo_cache::Row r;
        r.frame_idx = k.frame_idx;
        r.crop_x    = k.x;
        r.crop_y    = k.y;
        r.crop_w    = k.w;
        r.crop_h    = k.h;
        r.ppv       = kPpv;
        r.dets_json = "[]";
        r.ts_epoch  = 1700000000.0;
        rows.push_back(std::move(r));
    }

    try {
        db.put_many(rows);
    } catch (const std::exception& e) {
        std::cerr << "reader bench: put_many failed: " << e.what() << "\n";
        return false;
    }

    // Build the lookup workload — kReaderIter calls, ~50 % hit / miss.
    // Hit keys are sampled from `keys`; miss keys use frame_idx values
    // > 200 (guaranteed not inserted) so they always miss.
    std::vector<Key> lookups;
    lookups.reserve(kReaderIter);
    std::uniform_int_distribution<size_t> hit_idx_dist(0, keys.size() - 1);
    std::bernoulli_distribution           hit_or_miss(0.5);
    for (int i = 0; i < kReaderIter; ++i) {
        if (hit_or_miss(rng)) {
            lookups.push_back(keys[hit_idx_dist(rng)]);
        } else {
            Key k;
            k.frame_idx = 9000 + frame_dist(rng);  // > max inserted (199)
            k.x = coord_dist(rng) * 32;
            k.y = coord_dist(rng) * 24;
            k.w = kCellW;
            k.h = kCellH;
            lookups.push_back(k);
        }
    }

    // Warmup — 1k lookups to spin up SQLite's statement cache + file
    // page cache.
    for (int i = 0; i < kWarmupIter; ++i) {
        const Key& k = lookups[(size_t)i % lookups.size()];
        (void)db.get(k.frame_idx, k.x, k.y, k.w, k.h, kPpv);
    }

    // Hot loop — time each get() call individually.
    std::vector<double> samples_us;
    samples_us.reserve(kReaderIter);
    int hits = 0;
    for (const Key& k : lookups) {
        auto t0 = clock_t_::now();
        auto r  = db.get(k.frame_idx, k.x, k.y, k.w, k.h, kPpv);
        auto t1 = clock_t_::now();
        double dt_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        samples_us.push_back(dt_us);
        if (r.has_value()) ++hits;
    }

    db.close();
    ::unlink(db_path.c_str());
    ::unlink((db_path + "-wal").c_str());
    ::unlink((db_path + "-shm").c_str());

    auto pr = compute_percentiles(samples_us);
    const bool passed = pr.p99 < kReaderBarUs;

    std::cout << "[reader] hits=" << hits << " / " << kReaderIter << "\n";
    print_bench_result(std::cout, "reader (tile_cache_db::get)", pr, kReaderBarUs, passed);
    print_bench_result(report,    "reader (tile_cache_db::get)", pr, kReaderBarUs, passed);

    return passed;
}

// ---------------------------------------------------------------------------
// Writer bench — pad-probe based instrumentation of transform_ip.
//
// We attach two pad probes to the writer element:
//   * sink pad: GST_PAD_PROBE_TYPE_BUFFER — fires BEFORE transform_ip.
//                Record t0 in a thread-local-ish slot indexed by buffer
//                count.
//   * src pad : GST_PAD_PROBE_TYPE_BUFFER — fires AFTER transform_ip.
//                Compute dt = now - t0, append to the samples vector.
//
// Both probes run on the streaming thread (the same thread that runs
// transform_ip), so no synchronization between t0_recorded and the
// matching dt_computed is required: the sink probe runs, then
// transform_ip runs, then the src probe runs, all serially.
//
// We can't use a thread_local because pad probes run as callbacks; we
// stash the start timestamp in a struct shared by both probes.
// ---------------------------------------------------------------------------

struct WriterBenchCtx {
    clock_t_::time_point t0;
    std::vector<double>  samples_us;
};

static GstPadProbeReturn sink_probe_cb(GstPad* /*pad*/,
                                       GstPadProbeInfo* /*info*/,
                                       gpointer user_data) {
    auto* ctx = static_cast<WriterBenchCtx*>(user_data);
    ctx->t0 = clock_t_::now();
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn src_probe_cb(GstPad* /*pad*/,
                                      GstPadProbeInfo* /*info*/,
                                      gpointer user_data) {
    auto* ctx = static_cast<WriterBenchCtx*>(user_data);
    auto  t1  = clock_t_::now();
    double dt_us = std::chrono::duration<double, std::micro>(t1 - ctx->t0).count();
    ctx->samples_us.push_back(dt_us);
    return GST_PAD_PROBE_OK;
}

bool run_writer_bench(std::ostream& report) {
    const std::string out_db = std::string("/tmp/bench_writer_") +
                               std::to_string(::getpid()) + ".sqlite3";
    ::unlink(out_db.c_str());
    ::unlink((out_db + "-wal").c_str());
    ::unlink((out_db + "-shm").c_str());

    // Load the plugin from build tree (same env var the existing
    // hailocachewriter / reader tests use).
    const char* plugin_path_env = std::getenv("GST_HAILOCACHE_PLUGIN_PATH");
    std::string plugin_path = plugin_path_env
        ? plugin_path_env
        : std::string("../src/libgsthailocache.so");
    GError* err = nullptr;
    GstPlugin* plugin = gst_plugin_load_file(plugin_path.c_str(), &err);
    if (!plugin) {
        std::cerr << "writer bench: failed to load plugin at "
                  << plugin_path << ": "
                  << (err ? err->message : "(unknown)") << "\n";
        if (err) g_error_free(err);
        return false;
    }
    gst_object_unref(plugin);

    // Build the pipeline:
    //   videotestsrc num-buffers=N ! identity ! hailocachewriter ... ! fakesink
    //
    // identity sits before the writer so the videotestsrc → identity
    // hop is small; the pad probes on the writer sink/src pads bracket
    // ONLY the transform_ip call (plus an unavoidable handful of
    // nanoseconds of probe-callback overhead).
    const int N = kWriterIter + kWarmupIter;
    std::ostringstream desc;
    desc << "videotestsrc num-buffers=" << N
         << " is-live=false ! "
         << "video/x-raw,width=640,height=480,framerate=30/1 ! "
         << "hailocachewriter name=writer mode=tile_cache output-file="
         << out_db << " ! fakesink sync=false";
    err = nullptr;
    GstElement* pipeline = gst_parse_launch(desc.str().c_str(), &err);
    if (!pipeline) {
        std::cerr << "writer bench: parse_launch failed: "
                  << (err ? err->message : "(none)") << "\n";
        if (err) g_error_free(err);
        return false;
    }
    if (err) { g_error_free(err); err = nullptr; }

    GstElement* writer = gst_bin_get_by_name(GST_BIN(pipeline), "writer");
    if (!writer) {
        std::cerr << "writer bench: could not get 'writer' element\n";
        gst_object_unref(pipeline);
        return false;
    }
    GstPad* sink_pad = gst_element_get_static_pad(writer, "sink");
    GstPad* src_pad  = gst_element_get_static_pad(writer, "src");
    if (!sink_pad || !src_pad) {
        std::cerr << "writer bench: could not get writer pads\n";
        if (sink_pad) gst_object_unref(sink_pad);
        if (src_pad)  gst_object_unref(src_pad);
        gst_object_unref(writer);
        gst_object_unref(pipeline);
        return false;
    }

    WriterBenchCtx ctx;
    ctx.samples_us.reserve(N);

    gulong sink_probe = gst_pad_add_probe(sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                                          sink_probe_cb, &ctx, nullptr);
    gulong src_probe  = gst_pad_add_probe(src_pad,  GST_PAD_PROBE_TYPE_BUFFER,
                                          src_probe_cb,  &ctx, nullptr);

    GstStateChangeReturn sc = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (sc == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "writer bench: set_state(PLAYING) failed\n";
        gst_pad_remove_probe(sink_pad, sink_probe);
        gst_pad_remove_probe(src_pad,  src_probe);
        gst_object_unref(sink_pad);
        gst_object_unref(src_pad);
        gst_object_unref(writer);
        gst_object_unref(pipeline);
        return false;
    }

    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, 60 * GST_SECOND,
        (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

    bool pipeline_ok = false;
    if (!msg) {
        std::cerr << "writer bench: pipeline timed out\n";
    } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
        pipeline_ok = true;
    } else {
        GError* gerr = nullptr;
        gchar*  dbg  = nullptr;
        gst_message_parse_error(msg, &gerr, &dbg);
        std::cerr << "writer bench: pipeline error: "
                  << (gerr ? gerr->message : "?")
                  << " | debug: " << (dbg ? dbg : "") << "\n";
        if (gerr) g_error_free(gerr);
        g_free(dbg);
    }
    if (msg) gst_message_unref(msg);
    gst_object_unref(bus);

    // Read dropped-rows for diagnostics — if non-zero, the writer
    // thread couldn't keep up with the ring and we should report it.
    guint dropped = 0;
    g_object_get(writer, "dropped-rows", &dropped, nullptr);

    gst_element_set_state(pipeline, GST_STATE_NULL);

    gst_pad_remove_probe(sink_pad, sink_probe);
    gst_pad_remove_probe(src_pad,  src_probe);
    gst_object_unref(sink_pad);
    gst_object_unref(src_pad);
    gst_object_unref(writer);
    gst_object_unref(pipeline);

    ::unlink(out_db.c_str());
    ::unlink((out_db + "-wal").c_str());
    ::unlink((out_db + "-shm").c_str());

    if (!pipeline_ok) return false;

    // Drop the first kWarmupIter samples (warmup) before computing
    // percentiles.
    if ((int)ctx.samples_us.size() <= kWarmupIter) {
        std::cerr << "writer bench: only " << ctx.samples_us.size()
                  << " samples collected; expected > " << kWarmupIter << "\n";
        return false;
    }
    std::vector<double> hot(ctx.samples_us.begin() + kWarmupIter,
                            ctx.samples_us.end());

    auto pr = compute_percentiles(hot);
    const bool passed = pr.p99 < kWriterBarUs;

    std::cout << "[writer] samples=" << hot.size()
              << " dropped_rows=" << dropped << "\n";
    if (dropped > 0) {
        std::cout << "[writer] WARNING: " << dropped
                  << " rows dropped — ring overrun; numbers may be noisy.\n";
    }
    print_bench_result(std::cout, "writer (hailocachewriter::transform_ip)",
                       pr, kWriterBarUs, passed);
    print_bench_result(report,    "writer (hailocachewriter::transform_ip)",
                       pr, kWriterBarUs, passed);

    return passed;
}

// ---------------------------------------------------------------------------
// Report file resolution.
//
// We want the report to land at gst-hailo-cache/tests/bench-results.txt
// regardless of CWD. The bench binary is at
// gst-hailo-cache/build/tests/bench_lookup_latency, so the report goes
// to ../../tests/bench-results.txt relative to the binary. We resolve
// argv[0] via /proc/self/exe (Linux) to be CWD-independent.
// ---------------------------------------------------------------------------

std::string resolve_report_path() {
    char buf[4096];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) {
        // Fallback — CWD-relative.
        return "tests/bench-results.txt";
    }
    buf[n] = '\0';
    std::string exe = buf;
    // exe = .../gst-hailo-cache/build/tests/bench_lookup_latency
    // strip last 3 path components to get .../gst-hailo-cache/, then
    // append tests/bench-results.txt.
    for (int i = 0; i < 3; ++i) {
        auto pos = exe.find_last_of('/');
        if (pos == std::string::npos) {
            return "tests/bench-results.txt";
        }
        exe.resize(pos);
    }
    return exe + "/tests/bench-results.txt";
}

}  // namespace

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    const std::string report_path = resolve_report_path();
    std::ofstream report(report_path);
    if (!report) {
        std::cerr << "bench: could not open report file at "
                  << report_path << " — continuing with stdout only\n";
    } else {
        // Header.
        auto now   = std::chrono::system_clock::now();
        auto secs  = std::chrono::system_clock::to_time_t(now);
        report << "gst-hailo-cache lookup + writer microbench\n";
        report << "Plan 5 Task 10 — meson benchmark target\n";
        report << "Run timestamp (epoch s): " << (long long)secs << "\n";
        report << "Spec bars: reader p99 < " << kReaderBarUs << " us, "
               << "writer p99 < " << kWriterBarUs << " us\n";
        report << "Iterations: reader=" << kReaderIter
               << " (over " << kReaderDbRows << "-row DB, 50% hit), "
               << "writer=" << kWriterIter
               << " (after " << kWarmupIter << " warmup buffers)\n\n";
    }

    std::cout << "Running reader bench...\n";
    const bool reader_passed = run_reader_bench(report);
    std::cout << "Running writer bench...\n";
    const bool writer_passed = run_writer_bench(report);

    const bool overall = reader_passed && writer_passed;
    std::cout << "Overall: " << (overall ? "PASS" : "FAIL") << "\n";
    if (report) report << "Overall: " << (overall ? "PASS" : "FAIL") << "\n";

    return overall ? 0 : 1;
}
