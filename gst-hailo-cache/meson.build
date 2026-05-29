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
