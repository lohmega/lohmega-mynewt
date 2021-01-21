# BlueBerry from Loligoelectronics

Tiny sensornode which runs off a coincell

- nRF52 as MCU, i.e. ARM Cortex M4F.
- USB micro
- BLE
- Accelerometer/Gyro/Compass: MPU-9250.
- Altimeter: LPS22HB.
- Humidity sensor: HTS221
- UV / Ambient Sensor: Si1133


The source files are located in the src/ directory.

Header files are located in include/ 

pkg.yml contains the base definition of the package.

To erase the default flash image that shipped with the blueberry board.
```
$ JLinkExe -device nRF52 -speed 4000 -if SWD
J-Link>erase
J-Link>exit
$ 
```

```
newt target create blueberry_boot
newt target set blueberry_boot app=@apache-mynewt-core/apps/boot
newt target set blueberry_boot bsp=hw/bsp/blueberry
newt target set blueberry_boot build_profile=optimized
newt build blueberry_boot
newt create-image blueberry_boot 1.0.0
newt load blueberry_boot
```


```
newt target create bb_sensortest
newt target set bb_sensortest app=apps/sensor_test
newt target set bb_sensortest bsp=hw/bsp/blueberry
newt target set bb_sensortest build_profile=debug
newt run bb_sensortest 0
```

### Building for Nordic DFU

The important thing about running mynewt under the nordic dfu is that the FLASH start needs to match
what the bootloader expects.

```
# set the MYNEWT_VAL BOOT_NORDIC_DFU to 1
# build app
newt build bb_blinky

# Create intel-hex from elf
arm-none-eabi-objcopy -O ihex /home/niklas/git/lohmega/lohmega-mynewt-lps/bin/targets/bb_blinky/app/apps/blinky/blinky.elf hexs/bb_blinky.hex

# Generate dfu package
nrfutil pkg generate --debug-mode --hw-version 52 --sd-req 0x91,0x9e,0x9f --application hexs/bb_blinky.hex --application-version 1 --key-file ../berries/priv.pem hexs/bb_blinky_sdmult_dbg.zip

# Upload using dfu
nrfutil dfu ble -pkg hexs/bb_blinky_sdmult_dbg.zip -ic NRF52 -n "BBDfu 7641" -p /dev/ttyACM0
```