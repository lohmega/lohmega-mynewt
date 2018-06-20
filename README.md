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
newt run bb_sensortest 0.1.1

# and in a separate window run telnet to view the RTT output:
telnet localhost 19021

```