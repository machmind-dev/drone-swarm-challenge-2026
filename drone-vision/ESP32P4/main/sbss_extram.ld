/*
 * sbss_extram.ld  — Pre-empt the SRAM wildcard for large .sbss sections.
 *
 * Root cause:  The generated sections.ld has a wildcard *(.sbss .sbss.*) inside
 * .dram0.bss (SRAM). Every .sbss section ends up there.  With OpenCV + ArUco,
 * the C++ locale and newlib stdio singletons add ~403 KB of .sbss that exhausts
 * sram_low + sram_high, causing "--enable-non-contiguous-regions discards section"
 * errors for all remaining IDF component .sbss symbols.
 *
 * Fix: insert a PSRAM BSS section BEFORE .dram0.bss with explicit patterns for
 * the bulky system-library objects.  Explicit archive:object matches take priority
 * over the wildcard in .dram0.bss, so these sections land in ext_ram_seg (PSRAM)
 * and free the SRAM for the small IDF component .sbss symbols.
 *
 * RISC-V GP relaxation is optional — symbols too far from GP use auipc+addi
 * (PC-relative) automatically, so PSRAM placement is runtime-safe.
 */

SECTIONS
{
  .ext_ram.sbss_sys (NOLOAD) :
  {
    /* C++ locale facets: narrow and wide char — largest offenders (~350 KB) */
    libstdc++.a:cxx11-locale-inst.o (.sbss .sbss.* .bss .bss.*)
    libstdc++.a:cxx11-wlocale-inst.o(.sbss .sbss.* .bss .bss.*)

    /* newlib libc: env, stdio, timezone, atexit (~50 KB) */
    libc.a:libc_a-environ.o      (.sbss .sbss.* .bss .bss.*)
    libc.a:libc_a-findfp.o       (.sbss .sbss.* .bss .bss.*)
    libc.a:libc_a-tzset_r.o      (.sbss .sbss.* .bss .bss.*)
    libc.a:libc_a-tzvars.o       (.sbss .sbss.* .bss .bss.*)
    libc.a:libc_a-__atexit.o     (.sbss .sbss.* .bss .bss.*)
    libc.a:libc_a-__call_atexit.o(.sbss .sbss.* .bss .bss.*)
  } > ext_ram_seg
}
INSERT BEFORE .dram0.bss;
