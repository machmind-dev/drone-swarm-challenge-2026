/*
 * opencv_psram_bss.ld  (WiFi-DMA rescue linker fragment)
 *
 * Context
 * -------
 * CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y causes the ESP-IDF linker
 * template to generate an .ext_ram.bss output section (in extern_ram_seg /
 * PSRAM) that captures *all* .bss sections — including the WiFi MAC/PHY blobs
 * (libnet80211.a, libpp.a) which keep their DMA ring-buffer descriptors in BSS.
 * Placing those descriptors in PSRAM breaks WiFi DMA, causing TG1WDT_SYS_RST
 * immediately after "Init static tx buffer num: 16".
 *
 * Without CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY the PSRAM heap starts at
 * SOC_EXTRAM_DATA_LOW (0x3D000000) — the same address where the OpenCV BSS
 * sections are placed by an INSERT BEFORE .dram0.bss rule.  The heap's TLSF
 * metadata overwrites OpenCV BSS at boot, corrupting memory and causing the
 * same WiFi crash by a different path.
 *
 * Solution
 * --------
 * Keep CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y so _ext_ram_bss_end is
 * properly defined and esp_psram_extram_heap_add() starts the PSRAM heap AFTER
 * all BSS.  OpenCV BSS falls naturally into .ext_ram.bss (PSRAM).
 *
 * This fragment fires INSERT BEFORE .ext_ram.bss and captures the WiFi/LwIP
 * libraries' .bss sections into internal DRAM (dram0_0_seg) so they remain
 * DMA-reachable.
 *
 * Note on zero-initialisation
 * ---------------------------
 * The .wifi_dram.bss section is placed in dram0_0_seg but is NOT bracketed by
 * _bss_start/_bss_end (those symbols are defined inside the subsequent
 * .dram0.bss section).  The WiFi blob's esp_wifi_init() explicitly initialises
 * all internal state, so the initial BSS content does not matter.
 */

SECTIONS
{
  .wifi_dram.bss (NOLOAD) :
  {
    . = ALIGN(4);

    /* WiFi MAC / PHY blobs — DMA ring buffers and descriptors live here */
    *libnet80211.a:(.bss .bss.* COMMON)
    *libpp.a:(.bss .bss.* COMMON)

    /* WiFi component API layer */
    *libesp_wifi.a:(.bss .bss.* COMMON)

    /* WPA supplicant — authentication state */
    *libwpa_supplicant.a:(.bss .bss.* COMMON)

    /* WiFi / BT coexistence */
    *libcoexist.a:(.bss .bss.* COMMON)

    /* LwIP TCP/IP stack and network interface */
    *liblwip.a:(.bss .bss.* COMMON)
    *libesp_netif.a:(.bss .bss.* COMMON)

    . = ALIGN(4);
  } > dram0_0_seg
}
INSERT BEFORE .ext_ram.bss;
