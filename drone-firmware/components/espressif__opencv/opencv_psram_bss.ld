/*
 * opencv_psram_bss.ld
 *
 * Routes OpenCV library BSS/COMMON sections to PSRAM (extern_ram_seg) so they
 * do not consume the limited internal DRAM on ESP32-S3.
 *
 * Why INSERT BEFORE .dram0.bss (not .ext_ram.bss)
 * ------------------------------------------------
 * This script is the FIRST -T argument processed by the linker.  At that point
 * only the ROM linker files (-T esp32s3.rom.ld etc.) have been read, so only
 * memory regions defined there (including extern_ram_seg) are available.
 * dram0_0_seg is declared later in memory.ld and cannot be used here without
 * triggering an "implicit region" miscompile.
 *
 * INSERT BEFORE .dram0.bss fires just before the main DRAM-BSS output section,
 * by which time sections.ld / memory.ld have already been processed and all
 * memory regions are defined.  Sections consumed here are not re-consumed by
 * .dram0.bss, so WiFi / LwIP BSS naturally stays in DRAM — DMA-safe.
 *
 * Why CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y is also required
 * -------------------------------------------------------------------
 * Without that config, ESP-IDF's esp_psram_extram_heap_add() starts the PSRAM
 * heap at SOC_EXTRAM_DATA_LOW (0x3D000000) — the same address where .opencv.bss
 * is placed below.  The TLSF heap metadata would overwrite OpenCV BSS, causing
 * memory corruption that manifests as a WiFi crash during init.
 *
 * With the config set, esp_psram_extram_heap_add() uses _ext_ram_bss_end
 * (defined in the generated .ext_ram.bss section of sections.ld) as the heap
 * start.  .ext_ram.bss is placed after .opencv.bss in extern_ram_seg, so
 * _ext_ram_bss_end points just past our BSS — no overlap.
 *
 * Zero-initialisation
 * -------------------
 * .opencv.bss is placed before .ext_ram.bss in extern_ram_seg and is therefore
 * NOT covered by the [_ext_ram_bss_start, _ext_ram_bss_end] range that the
 * startup code zeros.  app_main() must zero [_opencv_psram_bss_start,
 * _opencv_psram_bss_end] explicitly before any OpenCV function is called.
 */

SECTIONS
{
  .opencv.bss (NOLOAD) :
  {
    _opencv_psram_bss_start = ABSOLUTE(.);
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
    _opencv_psram_bss_end = ABSOLUTE(.);
  } > extern_ram_seg
}
INSERT BEFORE .dram0.bss;
