# Provides the [scheme:extram_bss] linker fragment that routes BSS/common
# sections to PSRAM (extern_ram).  In projects that include esp_wifi this
# scheme is already defined in esp_wifi/linker.lf.  drone-vision has no
# WiFi, so we define it here to satisfy the opencv linker.lf mappings.
idf_component_register(LDFRAGMENTS "linker.lf")
