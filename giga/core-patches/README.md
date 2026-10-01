# Core patches (outside the repo - re-apply after any mbed_giga core reinstall)

Core: `arduino:mbed_giga` 4.6.0, files under
`%LOCALAPPDATA%\Arduino15\packages\arduino\hardware\mbed_giga\4.6.0\`

1. **libraries/Arduino_H7_Video/src/lv_conf_9.h** - enable `LV_FONT_MONTSERRAT_20/28/48`
   and raise `LV_MEM_SIZE` to `128 * 1024U`.
2. **libraries/Arduino_H7_Video/src/Arduino_H7_Video.cpp** - apply
   `Arduino_H7_Video-rotation-buffer.patch` (or run `scratchpad/rotfix.py` logic): the
   LVGL flush path `realloc()`ed its software-rotation buffer on EVERY flush with no
   NULL check. Heap fragmentation eventually made a realloc fail and the null
   destination hard-faulted inside `lv_draw_sw_rotate` mid-repaint - the mid-swipe
   crashes the hardware watchdog kept recovering from. The patch allocates once.

Apply with: `patch -p1 -d <core dir> < Arduino_H7_Video-rotation-buffer.patch`
