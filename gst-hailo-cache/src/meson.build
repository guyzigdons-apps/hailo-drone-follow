# libgsthailocache.so — single shared library registering the hailocache
# GStreamer plugin. Element classes (hailocachewriter, hailocachereader)
# land in later tasks; for now plugin.cpp is a no-op GST_PLUGIN_DEFINE
# so the pipeline / registry plumbing can be exercised end-to-end.

gsthailocache_sources = [
    'plugin.cpp',
]

shared_library('gsthailocache',
    gsthailocache_sources,
    cpp_args      : ['-DHAVE_CONFIG_H'],
    dependencies  : [gst_dep, gst_base_dep, gst_video_dep, sqlite3_dep],
    install       : true,
    install_dir   : gst_plugins_dir,
)
