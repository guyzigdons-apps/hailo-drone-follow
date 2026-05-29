# libgsthailocache.so — single shared library registering the hailocache
# GStreamer plugin. Element classes (hailocachewriter, hailocachereader)
# land in later tasks; tile_cache_db.{hpp,cpp} (Plan 5 Task 2) holds the
# shared SQLite I/O code consumed by both elements and by the unit
# tests under ../tests.

gsthailocache_sources = [
    'plugin.cpp',
    'tile_cache_db.cpp',
    'cache_keys.cpp',
    'gst_hailocachewriter.cpp',
    'gst_hailocachereader.cpp',
]

gsthailocache_inc = include_directories('.')

gsthailocache_lib = shared_library('gsthailocache',
    gsthailocache_sources,
    include_directories : gsthailocache_inc,
    cpp_args      : ['-DHAVE_CONFIG_H'],
    dependencies  : [gst_dep, gst_base_dep, gst_video_dep, sqlite3_dep],
    install       : true,
    install_dir   : gst_plugins_dir,
)
