# loligo-mynewt
Apache Mynewt BSP and Drivers for Loligo boards

## Example: Sensor test app with Blueberry

The testapp sensor_test under apps can be run using the following procedure:

First make sure the blueberry device is erased:

```

nrfjprog -f NRF52 -e

```

Build and load the bootloader:

```
newt build blueberry_boot
newt load blueberry_boot

```

Build and run the application (build is automatic with run):

```
newt run blueberry_sensor_test 0.1.1

# and in a separate window run telnet to view the RTT output:
telnet localhost 19021

```

And you should see output in telnet similar to:

```
Trying 127.0.0.1...
Connected to localhost.
Escape character is '^]'.
SEGGER J-Link V6.16g - Real time terminal output
J-Link OB-SAM3U128-V2-NordicSemi compiled Jul 24 2017 17:30:12 V1.0, SN=682656561
Process: JLinkGDBServer
000128 accel (m/s^2) x = -5.300762656 y = -2.398990144 z = 8.283937440
000128 gyro (deg/s)  x = -0.365853664 y = -0.060975612 z = 2.317073120 
000130 pressure = 101612.01562 Pa
000130 relative humidity = 42.799999232%rh
000130 temperature = 22.924999232 Deg C
000770 accel (m/s^2) x = -5.319916256 y = -2.384624960 z = 8.303091040
000770 gyro (deg/s)  x = -0.487804896 y = 0.000000000 z = 2.378048896 
000772 pressure = 101612.37500 Pa
000772 relative humidity = 42.500000000%rh
000772 temperature = 23.215999600 Deg C
...

```