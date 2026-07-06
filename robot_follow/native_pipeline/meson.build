# Standalone meson project for the drone-follow native detection pipeline.
# Cross-compiles against the Hailo Yocto Poky SDK. See build.sh for the
# wrapper that sources the SDK environment and invokes meson + ninja.

project('drone-follow-native-pipeline', 'cpp',
    version : '0.1.0',
    default_options : [ 'warning_level=2', 'cpp_std=c++20', 'buildtype=debug' ])

# All dependencies come from the SDK sysroot via pkg-config. The
# Hailo Yocto SDK installs .pc files for each of these in
# /opt/poky/<ver>/sysroots/armv8a-poky-linux/usr/lib/pkgconfig/.
hailo_analytics_dep = dependency('hailo-analytics', required : true)
medialib_dep        = dependency('hailo-medialib', required : true)
gst_dep             = dependency('gstreamer-1.0', version : '>=1.0.0')
gst_app_dep         = dependency('gstreamer-app-1.0', version : '>=1.0.0')
spdlog_dep          = dependency('spdlog', version : '>=1.9.2')
# cppzmq is header-only (zmq.hpp ships in the sysroot include dir); we
# only need the underlying libzmq to link against. nlohmann_json is also
# header-only — its pkg-config sets the include path and adds no libs.
zmq_dep             = dependency('libzmq', required : true)
nlohmann_json_dep   = dependency('nlohmann_json', required : true)

executable('df_native_pipeline',
    ['main.cpp', 'drone_follow_pipeline_builder.cpp', 'dynamic_tiling_stage.cpp',
     'tile_toggler.cpp'],
    dependencies : [
        hailo_analytics_dep,
        medialib_dep,
        gst_dep,
        gst_app_dep,
        spdlog_dep,
    ],
    gnu_symbol_visibility : 'default',
    install : true,
    install_dir : 'bin')
