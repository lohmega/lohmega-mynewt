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