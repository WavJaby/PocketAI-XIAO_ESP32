idf_component_register(SRCS "main.c" "lv_port_disp.c" "camera.c" "atomic.c" "gui/main_ui.c"
        PRIV_REQUIRES lvgl esp_lcd nvs_flash esp32-camera
        INCLUDE_DIRS .
)

# Patch cam_hal.c to remove static from cam_obj and add getter
file(READ ${project_dir}/managed_components/espressif__esp32-camera/driver/cam_hal.c LIB_CONTENT)

string(REPLACE "static cam_obj_t *cam_obj = NULL;" "cam_obj_t *cam_obj = NULL;" LIB_CONTENT "${LIB_CONTENT}")
string(REPLACE "static int cam_verify_jpeg_eoi"
        "int cam_verify_jpeg_eoi" LIB_CONTENT "${LIB_CONTENT}")
string(REPLACE "static bool cam_get_next_frame"
        "bool cam_get_next_frame(int *frame_pos);\n\
bool cam_get_next_frame_old" LIB_CONTENT "${LIB_CONTENT}")

file(WRITE ${project_dir}/managed_components/espressif__esp32-camera/driver/cam_hal.c "${LIB_CONTENT}")
file(REMOVE ${project_dir}/managed_components/espressif__esp32-camera/.component_hash)

## Add patched file as a library
#add_library(patched_cam_hal STATIC ${CMAKE_BINARY_DIR}/patched_cam_hal.c)
#
#
## Include directories for esp32-camera
#target_include_directories(patched_cam_hal PRIVATE
#        ${idf_path}/components/esp_driver_ledc/include
#        ${idf_path}/components/esp_driver_gpio/include
#        ${project_dir}/managed_components/espressif__esp32-camera/conversions/include
#        ${project_dir}/managed_components/espressif__esp32-camera/driver/include
#        ${project_dir}/managed_components/espressif__esp32-camera/driver/private_include
#        ${project_dir}/managed_components/espressif__esp32-camera/target/private_include
#)
#
## Link patched library to main component
#idf_component_get_property(main_lib main COMPONENT_LIB)
#target_link_libraries(${main_lib} PRIVATE patched_cam_hal)