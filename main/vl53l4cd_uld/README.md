# VL53L4CD Ultra Lite Driver

`vl53l4cd_api.c` and `vl53l4cd_api.h` are STMicroelectronics VL53L4CD ULD
version 2.2.2.0, copied from the official `x-cube-tof1` repository. `platform.h`
and `../vl53l4cd_platform.c` provide the ESP-IDF I2C port. A zero-SPAD guard was
added to `VL53L4CD_GetResult` to avoid division by zero on an incomplete sample.

See `ST-LICENSE.md` for the upstream license notice.
