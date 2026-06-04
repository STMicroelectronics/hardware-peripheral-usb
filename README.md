# hardware-usb #

This module contains the STMicroelectronics android.hardware.usb and android.hardware.usb.gadget source code.
It is part of the STMicroelectronics delivery for Android.

## Description ##

This module implements android.hardware.usb AIDL version 1 and android.hardware.usb.gadget AIDL version 1.
Please see the Android delivery release notes for more details.

## Documentation ##

* The [release notes][] provide information on the release.
[release notes]: https://wiki.st.com/stm32mpu/wiki/STM32_MPU_OpenSTDroid_release_note_-_v6.2.0

## Dependencies ##

This module can't be used alone. It is part of the STMicroelectronics delivery for Android.

For USB and USB gadget:
```
PRODUCT_PACKAGES += \
    android.hardware.usb-service.stm32mpu
    android.hardware.usb.gadget-service.stm32mpu
```

## Contents ##

This directory contains the sources and associated Android build files to generate the USB and USB gadget binaries.

## License ##

This module is distributed under the Apache License, Version 2.0 found in the [LICENSE](./LICENSE) file.
