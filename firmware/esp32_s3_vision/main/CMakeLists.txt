idf_component_register(SRCS "app_main.c" "mjpeg_dht.c" "aruco_detect.c" "aruco_pose.c" "vision_frame.c"
                       INCLUDE_DIRS ".")

# Detection runs per pixel on every frame: build it for speed whatever the project setting is.
set_source_files_properties(aruco_detect.c aruco_pose.c PROPERTIES COMPILE_OPTIONS "-O2")
