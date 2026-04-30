/*
 * sbss_extram.ld — Route .bss/.sbss from OpenCV and toolchain libs to PSRAM.
 *
 * Replaces sbss_psram.lf for IDF 5.5+.
 *
 * Problem: IDF 5.5 changed the ldgen template so that .dram0.bss no longer
 * emits EXCLUDE_FILE() patterns for extern_ram archive mappings. The wildcard
 * *(.bss .bss.*) in .dram0.bss grabs all BSS before .ext_ram.bss can claim it.
 * Result: sbss_psram.lf entries are present in sections.ld but .ext_ram.bss
 * stays at size 0 → sram_seg overflows by ~44 KB.
 *
 * Fix: INSERT a new PSRAM-bound output section BEFORE .dram0.bss using raw
 * GNU ld ARCHIVE:OBJECT wildcard syntax. Explicit archive patterns are consumed
 * first and never reach the wildcard in .dram0.bss.
 *
 * Requirement: CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y (sdkconfig.defaults)
 * must be active so the linker script exports ext_ram_seg. Without it this
 * section has no destination region and the link fails.
 *
 * Safety note: .sdata (GP-relative initialised data) is intentionally excluded.
 * RISC-V .sdata holds GP-relative *initialised* data including function-pointer
 * dispatch tables (e.g. CvtFunc linear_tab[] in cvtColor). If .sdata lives in
 * PSRAM, camera DMA bus pressure can corrupt those pointers mid-read → garbage
 * PC → instruction access fault. Keep .sdata in SRAM.
 */

SECTIONS
{
  .ext_ram.bss_libs (NOLOAD) :
  {
    /* OpenCV core modules */
    *libopencv_core.a:*(.bss .bss.* .sbss .sbss.*)
    *libopencv_imgproc.a:*(.bss .bss.* .sbss .sbss.*)
    *libopencv_objdetect.a:*(.bss .bss.* .sbss .sbss.*)
    *libopencv_features2d.a:*(.bss .bss.* .sbss .sbss.*)
    *libopencv_calib3d.a:*(.bss .bss.* .sbss .sbss.*)
    *libopencv_flann.a:*(.bss .bss.* .sbss .sbss.*)

    /* C++ standard library (locale facets, iostream — ~350 KB of BSS) */
    *libstdc++.a:*(.bss .bss.* .sbss .sbss.*)

    /* newlib libc (env, stdio, timezone, atexit — ~50 KB of BSS) */
    *libc.a:*(.bss .bss.* .sbss .sbss.*)
  } > ext_ram_seg
}
INSERT BEFORE .dram0.bss;
