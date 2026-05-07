/*
 * opencv_psram_bss.ld — intentionally empty.
 *
 * OpenCV BSS routing to PSRAM is now handled by linker.lf (ESP-IDF linker
 * fragment mechanism), which places OpenCV BSS inside the .ext_ram.bss output
 * section so that esp_psram.c correctly accounts for it when computing the
 * PSRAM heap start.
 *
 * See linker.lf for the mapping definitions.
 */
