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
