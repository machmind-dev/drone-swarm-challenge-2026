/*
 * opencv_psram_bss.ld
 *
 * Routes OpenCV library BSS/COMMON sections to PSRAM (extern_ram_seg) so they
 * do not consume the limited internal DRAM on ESP32-S3.
 *
 * Why INSERT BEFORE .ext_ram.bss
 * --------------------------------
 * On ESP32-S3, extern_ram_seg and drom0_0_seg (flash) share the same MMU bus
 * and address space, both starting at 0x3C000020.  The generated sections.ld
 * contains a .ext_ram.dummy (NOLOAD) section that advances the location
 * counter past the flash-mapped DROM content to the first real PSRAM address
 * (e.g. 0x3C5A0000) before .ext_ram.bss is placed.
 *
 * INSERT BEFORE .dram0.bss fires before .ext_ram.dummy has run, so the
 * extern_ram_seg pointer is still at 0x3C000020 — the start of the read-only
 * flash region.  Any memset to that address faults immediately.
 *
 * INSERT BEFORE .ext_ram.bss fires after .ext_ram.dummy has advanced the
 * pointer to actual PSRAM.  Our .opencv.bss then occupies the first N bytes
 * of real PSRAM, followed immediately by .ext_ram.bss.
 *
 * Memory layout in extern_ram_seg (PSRAM address space)
 * -------------------------------------------------------
 *   [0x3C000020 .. flash_end]      .ext_ram.dummy  — placeholder, skips flash
 *   [psram_start .. psram_start+N] .opencv.bss     — OpenCV BSS (this script)
 *   [psram_start+N .. +M]          .ext_ram.bss    — WiFi/LwIP/app PSRAM BSS
 *   [psram_start+N+M .. end]       PSRAM heap      — starts at _ext_ram_bss_end
 *
 * Why CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y is required
 * --------------------------------------------------------------
 * This config causes ESP-IDF to emit .ext_ram.bss with _ext_ram_bss_start/end
 * symbols and to start the PSRAM heap at _ext_ram_bss_end.  Without it the
 * heap starts at the raw PSRAM base, overlapping our BSS.
 *
 * Zero-initialisation
 * -------------------
 * .opencv.bss is placed BEFORE .ext_ram.bss in extern_ram_seg, so it is NOT
 * covered by the [_ext_ram_bss_start, _ext_ram_bss_end] range that the startup
 * code zeros.  app_main() must call memset on [_opencv_psram_bss_start,
 * _opencv_psram_bss_end] before any OpenCV function is invoked.
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
INSERT BEFORE .ext_ram.bss;
