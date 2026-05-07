#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * vision_bench_start()
 *
 * Launches the multi-resolution ArUco benchmark as two pinned FreeRTOS tasks:
 *   - "bench_cam"  Core 0  — drains camera DMA, downsamples, forwards frames
 *   - "bench_det"  Core 1  — runs ArUco detectMarkers + solvePnP, logs stats
 *
 * Benchmark stages (camera is reinitialised between stages):
 *   Stage 0 : 160 × 120   QQVGA   — baseline, fits in internal DRAM
 *   Stage 1 : 320 × 240   QVGA    — 4× pixels, standard ArUco use-case
 *   Stage 2 : 480 × 320   HVGA    — 9× pixels, OV3660 sweet-spot candidate
 *   Stage 3 : 640 × 480   VGA     — 16× pixels, likely PSRAM-limited
 *
 * After all stages the results are printed as a CSV table to UART0.
 */
void vision_bench_start(void);

#ifdef __cplusplus
}
#endif
