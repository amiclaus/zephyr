# Copyright (c) 2026 Analog Devices Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ANALOG_SWITCH_ADGM3121 adgm3121.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_SWITCH_ADG1736  adg1736.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_SWITCH_ADG1712  adg1712.c)
