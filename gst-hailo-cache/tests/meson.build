# Tests for gst-hailo-cache.
#
# Plan 5 Task 2: C++ gtest unit tests for tile_cache_db, plus a tiny
# `cache_db_cli` helper binary used by the Python bit-exact round-trip
# test (tests/integration/test_cache_cpp_round_trip.py) — the CLI is
# test-only and is NOT installed.

gtest_dep = dependency('gtest', main : true, required : true)

# Link against the shared plugin lib so we exercise exactly the same
# binary that ships. This avoids the ODR risk of recompiling the
# .cpp twice (once into the .so, once into the test exe).
test_tile_cache_db = executable('test_tile_cache_db',
    'test_tile_cache_db.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [gtest_dep, sqlite3_dep],
    link_with           : gsthailocache_lib,
    install             : false,
)

test('tile_cache_db', test_tile_cache_db, protocol : 'gtest')

# cache_db_cli — test-only helper that lets a Python test write rows via
# the C++ TileCacheDb::put_many path and then read them back through
# hailo_tiling.cache.SqliteCacheStore. NOT installed.
cache_db_cli = executable('cache_db_cli',
    'cache_db_cli.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [sqlite3_dep],
    link_with           : gsthailocache_lib,
    install             : false,
)

# Plan 5 Task 3 — cache_keys gtest + Python parity CLI.
test_keys = executable('test_keys',
    'test_keys.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [gtest_dep, gst_dep],
    link_with           : gsthailocache_lib,
    install             : false,
)

test('test_keys', test_keys, protocol : 'gtest')

# cache_keys_cli — test-only helper used by
# tests/integration/test_cache_keys_python_parity.py. NOT installed.
cache_keys_cli = executable('cache_keys_cli',
    'cache_keys_cli.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [gst_dep],
    link_with           : gsthailocache_lib,
    install             : false,
)

# Plan 5 Task 4 — hailocachewriter element gtest.
#
# Unlike test_tile_cache_db / test_keys (which link directly against the
# .so and exercise internal symbols), this test must EXERCISE the loaded
# GStreamer plugin — element-factory lookups need the plugin registered
# with the global gstreamer registry, which happens via
# gst_plugin_load_file(). The test passes the path through env var
# GST_HAILOCACHE_PLUGIN_PATH; meson.build below sets it relative to the
# build tree so `meson test` works straight after `ninja`.
test_hailocachewriter = executable('test_hailocachewriter',
    'test_hailocachewriter.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [gtest_dep, gst_dep, gst_base_dep, sqlite3_dep],
    install             : false,
)

test('hailocachewriter', test_hailocachewriter,
     protocol : 'gtest',
     env      : [
         'GST_HAILOCACHE_PLUGIN_PATH=' + gsthailocache_lib.full_path(),
     ])

# Plan 5 Task 8 — hailocachereader element gtest.
#
# Same loading pattern as the writer: the test binary loads
# libgsthailocache.so explicitly via gst_plugin_load_file using the
# GST_HAILOCACHE_PLUGIN_PATH env var. This avoids depending on a
# system-installed copy of the plugin.
test_hailocachereader = executable('test_hailocachereader',
    'test_hailocachereader.cpp',
    include_directories : gsthailocache_inc,
    dependencies        : [gtest_dep, gst_dep, gst_base_dep],
    install             : false,
)

test('hailocachereader', test_hailocachereader,
     protocol : 'gtest',
     timeout  : 30,
     env      : [
         'GST_HAILOCACHE_PLUGIN_PATH=' + gsthailocache_lib.full_path(),
     ])
