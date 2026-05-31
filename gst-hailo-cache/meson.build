project('gst-hailo-cache', 'c', 'cpp',
        version : '0.1.0',
        meson_version : '>= 0.59',
        default_options : [ 'warning_level=1',
                            'buildtype=release',
                            'c_std=c11', 'cpp_std=c++17',
                            'c_args=-Wno-psabi',
                            'cpp_args=-Wno-psabi -Wno-class-memaccess -Wno-deprecated-declarations']
       )

gst_dep      = dependency('gstreamer-1.0',       required : true)
gst_base_dep = dependency('gstreamer-base-1.0',  required : true)
gst_video_dep = dependency('gstreamer-video-1.0', required : true)
sqlite3_dep  = dependency('sqlite3',             required : true)

# TAPPAS core: provides libgsthailometa (get_hailo_main_roi) plus the
# hailo_objects.hpp / gst_hailo_meta.hpp headers used by the writer's
# crop-provenance path (Phase 14). Mirror how
# hailo-apps/hailo_apps/postprocess/meson.build finds it. Optional so a
# no-Hailo dev box can still build the cache lib (the writer then falls
# back to the full-frame crop — see gst_hailocachewriter.cpp).
tappas_dep = dependency('hailo-tappas-core', version : '>=3.30.0', required : false)
if not tappas_dep.found()
    tappas_dep = dependency('hailo_tappas', version : '>=3.30.0', required : false)
endif
have_tappas = tappas_dep.found()
message('hailo-tappas-core (crop provenance): @0@'.format(have_tappas))

# Resolve the system GStreamer plugin directory the same way
# hailo-apps/hailo_apps/postprocess/meson.build does.
gst_plugins_dir = '/usr/lib/gstreamer-1.0'
gst_plugins_dir_result = run_command('pkg-config', '--variable=pluginsdir', 'gstreamer-1.0', check: false)
if gst_plugins_dir_result.returncode() == 0
    gst_plugins_dir = gst_plugins_dir_result.stdout().strip()
endif
message('GStreamer plugin install dir: @0@'.format(gst_plugins_dir))

with_bench = get_option('with_bench')
message('with_bench: @0@'.format(with_bench))

subdir('src')
subdir('tests')
