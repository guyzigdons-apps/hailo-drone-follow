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
    'gst_hailocachebypass.cpp',
]

gsthailocache_inc = include_directories('.')

# Crop-provenance (Phase 14): when TAPPAS core is available, link
# libgsthailometa + pick up hailo_objects.hpp / gst_hailo_meta.hpp so the
# writer can read each tile-crop's HailoROI bbox. Guard the code behind
# HAVE_GSTHAILOMETA so the lib still builds on a no-Hailo box (writer then
# records the full-frame fallback crop).
gsthailocache_deps = [gst_dep, gst_base_dep, gst_video_dep, sqlite3_dep]
gsthailocache_cpp_args = ['-DHAVE_CONFIG_H']
if have_tappas
    gsthailocache_deps += tappas_dep
    gsthailocache_cpp_args += '-DHAVE_GSTHAILOMETA=1'
endif

gsthailocache_lib = shared_library('gsthailocache',
    gsthailocache_sources,
    include_directories : gsthailocache_inc,
    cpp_args      : gsthailocache_cpp_args,
    dependencies  : gsthailocache_deps,
    install       : true,
    install_dir   : gst_plugins_dir,
)
