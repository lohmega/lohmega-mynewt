
Sensirion SGP40 Volatile Organic Compound (VOC) sensor
======================================================
renamed and copied soureces.


sources from https://github.com/Sensirion/embedded-sgp/tree/master/sgp40 release 7.1.1:

```
cp sgp40-7.1.1/sgp40.c sgp40/src/sensirion_sgp40.c
cp sgp40-7.1.1/sgp40.h sgp40/src/sensirion_sgp40.h
cp sgp40-7.1.1/sgp_git_version.h sgp40/src/
```

sources from https://github.com/Sensirion/embedded-common/ release 7.1.1:
```
cp sgp40-7.1.1/sensirion_common.h sgp40/src/
cp sgp40-7.1.1/sensirion_i2c.h sgp40/src/
cp sgp40-7.1.1/sensirion_arch_config.h sgp40/src/
cp sgp40-7.1.1/sensirion_common.c sgp40/src/
```

sgp40/src/sensirion_i2c.c  <-- this is the mynewt i2c port


sources from https://github.com/Sensirion/embedded-sgp/tree/master/sgp40_voc_index
(commit e9ecd60fe7031b05e9ac0ddd7445ade8ee177d7e):
```
sensirion_voc_algorithm.h 
sensirion_voc_algorithm.c 
```

Notes
=====
- add self test at init?

- idle mode is activated after power-up, after calling the sgp40_heater_off,or
  after a soft reset power consumption in idle mode is 34 uA (typ.)
