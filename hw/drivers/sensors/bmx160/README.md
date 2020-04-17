There is currently (2020Q2) no BMX160 driver from bosch. However, the BMX160
driver can be achived with the BMI160 and BMM150 drivers with some modifications.

Possible incomplete list of modifications required:
- change excpected chip_id to 0xD8
- remove writes to Reserved register 0x4B (i2c addres on BMI160, "Reserved" on BMX160). might not be needed.


Notes: 

- What is refered to as "MAG" (magnetic) in bmx160 manual is the same as "AUX" in BMI160 driver/manual.

- the BMX160 can be seen as having a BMM150 chip onboard which is accesible
  throught the BMX160 "MAG_..." registers.

References:
https://github.com/BoschSensortec/BMI160_driver
https://github.com/BoschSensortec/BMM150-Sensor-API

https://github.com/zephyrproject-rtos/zephyr/tree/master/drivers/sensor/bmi160
https://github.com/zephyrproject-rtos/zephyr/tree/master/drivers/sensor/bmm150

https://android.googlesource.com/kernel/mediatek/+/android-mediatek-sprout-3.10-marshmallow-mr1/drivers/misc/mediatek/magnetometer/bmm150/


