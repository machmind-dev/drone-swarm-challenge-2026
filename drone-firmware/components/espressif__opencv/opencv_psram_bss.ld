/*
 * opencv_psram_bss.ld
 *
 * Routes all BSS / COMMON data from OpenCV static archives to PSRAM
 * (extern_ram_seg) so it does not overflow the ~345 KB internal DRAM on
 * ESP32-S3.
 *
 * This script is appended to the ESP-IDF link command via
 * idf_build_set_property(LINK_OPTIONS ...) in the component CMakeLists.txt.
 * GNU ld INSERT BEFORE guarantees these patterns are matched before the
 * .dram0.bss section consumes them into internal DRAM.
 */

SECTIONS
{
  .opencv.bss (NOLOAD) :
  {
    . = ALIGN(4);

    /* OpenCV modules */
    *libopencv_core.a:(.bss .bss.* COMMON)
    *libopencv_imgproc.a:(.bss .bss.* COMMON)
    *libopencv_objdetect.a:(.bss .bss.* COMMON)
    *libopencv_calib3d.a:(.bss .bss.* COMMON)
    *libopencv_features2d.a:(.bss .bss.* COMMON)
    *libopencv_flann.a:(.bss .bss.* COMMON)

    /* OpenCV bundled 3rd-party libraries */
    *libzlib.a:(.bss .bss.* COMMON)
    *liblibjpeg-turbo.a:(.bss .bss.* COMMON)

    . = ALIGN(4);
  } > extern_ram_seg
}
INSERT BEFORE .dram0.bss;
