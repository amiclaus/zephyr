# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(adgm3121.c)
zephyr_library_sources_ifdef(CONFIG_EMUL adgm3121_emul.c)
