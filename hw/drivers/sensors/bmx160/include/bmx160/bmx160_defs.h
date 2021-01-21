#ifndef __BMX160_DEFS_H__
#define __BMX160_DEFS_H__

#define BMX160_REG_CHIP_ID                  (0x00)
#define BMX160_REG_ERROR                    (0x02)
#define BMX160_REG_PMU_STATUS               (0x03)
#define BMX160_REG_MAG_DATA                 (0x04) // s16 * xyz
#define BMX160_REG_RHALL_DATA               (0x0A) // u16
#define BMX160_REG_GYR_DATA                 (0x0C) // s16 * xyz
#define BMX160_REG_ACC_DATA                 (0x12) // s16 * xyz
#define BMX160_REG_STATUS                   (0x1B)
#define BMX160_REG_INT_STATUS               (0x1C)
#define BMX160_REG_FIFO_LENGTH              (0x22)
#define BMX160_REG_FIFO_DATA                (0x24)
#define BMX160_REG_ACC_CONF                 (0x40)
#define BMX160_REG_ACC_RANGE                (0x41)
#define BMX160_REG_GYR_CONF                 (0x42)
#define BMX160_REG_GYR_RANGE                (0x43)
#define BMX160_REG_MAG_CONF                 (0x44)
#define BMX160_REG_FIFO_DOWN                (0x45)
#define BMX160_REG_FIFO_CONF_0              (0x46)
#define BMX160_REG_FIFO_CONF_1              (0x47)
#define BMX160_REG_MAG_IF_0_CFG             (0x4C) // MAG_IF_0
#define BMX160_REG_MAG_IF_1_RDA             (0x4D) // MAG_IF_1
#define BMX160_REG_MAG_IF_2_WRA             (0x4E) // MAG_IF_2
#define BMX160_REG_MAG_IF_3_WRD             (0x4F) // MAG_IF_3
#define BMX160_REG_INT_ENABLE_0             (0x50)
#define BMX160_REG_INT_ENABLE_1             (0x51)
#define BMX160_REG_INT_ENABLE_2             (0x52)
#define BMX160_REG_INT_OUT_CTRL             (0x53)
#define BMX160_REG_INT_LATCH                (0x54)
#define BMX160_REG_INT_MAP_0                (0x55)
#define BMX160_REG_INT_MAP_1                (0x56)
#define BMX160_REG_INT_MAP_2                (0x57)
#define BMX160_REG_INT_DATA_0               (0x58)
#define BMX160_REG_INT_DATA_1               (0x59)
#define BMX160_REG_INT_LOWHIGH_0            (0x5A)
#define BMX160_REG_INT_LOWHIGH_1            (0x5B)
#define BMX160_REG_INT_LOWHIGH_2            (0x5C)
#define BMX160_REG_INT_LOWHIGH_3            (0x5D)
#define BMX160_REG_INT_LOWHIGH_4            (0x5E)
#define BMX160_REG_INT_MOTION_0             (0x5F)
#define BMX160_REG_INT_MOTION_1             (0x60)
#define BMX160_REG_INT_MOTION_2             (0x61)
#define BMX160_REG_INT_MOTION_3             (0x62)
#define BMX160_REG_INT_TAP_0                (0x63)
#define BMX160_REG_INT_TAP_1                (0x64)
#define BMX160_REG_INT_ORIENT_0             (0x65)
#define BMX160_REG_INT_ORIENT_1             (0x66)
#define BMX160_REG_INT_FLAT_0               (0x67)
#define BMX160_REG_INT_FLAT_1               (0x68)
#define BMX160_REG_FOC_CONF                 (0x69)
#define BMX160_REG_NVM_CONF                 (0x6A)
#define BMX160_REG_IF_CONF                  (0x6B)
#define BMX160_REG_SELF_TEST                (0x6D)
#define BMX160_REG_OFFSET                   (0x71)
#define BMX160_REG_OFFSET_CONF              (0x77)
#define BMX160_REG_INT_STEP_CNT_0           (0x78)
#define BMX160_REG_INT_STEP_CONFIG_0        (0x7A)
#define BMX160_REG_INT_STEP_CONFIG_1        (0x7B)
#define BMX160_REG_CMD                      (0x7E)
#define BMX160_REG_SPI_COMM_TEST            (0x7F)
//#define BMX160_REG_INTL_PULLUP_CONF         UINT8_C(0x85)
//
//

#define BMX160_STATUS_DRDY_ACC         (1 << 7)
#define BMX160_STATUS_DRDY_GYR         (1 << 6)
#define BMX160_STATUS_DRDY_MAG         (1 << 5)
#define BMX160_STATUS_NVM_RDY          (1 << 4)
#define BMX160_STATUS_FOC_RDY          (1 << 3)
#define BMX160_STATUS_MAG_MAN_OP       (1 << 2)
#define BMX160_STATUS_GYR_SELF_TEST_OK (1 << 1)


/* Accel Output data rate */
#define BMX160_ACC_CONF_ODR_0_78HZ              (0x01)
#define BMX160_ACC_CONF_ODR_1_56HZ              (0x02)
#define BMX160_ACC_CONF_ODR_3_12HZ              (0x03)
#define BMX160_ACC_CONF_ODR_6_25HZ              (0x04)
#define BMX160_ACC_CONF_ODR_12_5HZ              (0x05)
#define BMX160_ACC_CONF_ODR_25HZ                (0x06)
#define BMX160_ACC_CONF_ODR_50HZ                (0x07)
#define BMX160_ACC_CONF_ODR_100HZ               (0x08)
#define BMX160_ACC_CONF_ODR_200HZ               (0x09)
#define BMX160_ACC_CONF_ODR_400HZ               (0x0A)
#define BMX160_ACC_CONF_ODR_800HZ               (0x0B)
#define BMX160_ACC_CONF_ODR_1600HZ              (0x0C)
/* Accel bandwidth */
#define BMX160_ACC_CONF_BWP_OSR4_AVG1            (0x00 << 4)
#define BMX160_ACC_CONF_BWP_OSR2_AVG2            (0x01 << 4)
#define BMX160_ACC_CONF_BWP_NORMAL_AVG4          (0x02 << 4)
#define BMX160_ACC_CONF_BWP_RES_AVG8             (0x03 << 4)
#define BMX160_ACC_CONF_BWP_RES_AVG16            (0x04 << 4)
#define BMX160_ACC_CONF_BWP_RES_AVG32            (0x05 << 4)
#define BMX160_ACC_CONF_BWP_RES_AVG64            (0x06 << 4)
#define BMX160_ACC_CONF_BWP_RES_AVG128           (0x07 << 4)

/* Accel Range */
#define BMX160_ACC_RANGE_2G                (0x03)
#define BMX160_ACC_RANGE_4G                (0x05)
#define BMX160_ACC_RANGE_8G                (0x08)
#define BMX160_ACC_RANGE_16G               (0x0C)

/* Gyro Output Data Rate */
#define BMX160_GYR_CONF_ODR_25HZ                 (0x06)
#define BMX160_GYR_CONF_ODR_50HZ                 (0x07)
#define BMX160_GYR_CONF_ODR_100HZ                (0x08)
#define BMX160_GYR_CONF_ODR_200HZ                (0x09)
#define BMX160_GYR_CONF_ODR_400HZ                (0x0A)
#define BMX160_GYR_CONF_ODR_800HZ                (0x0B)
#define BMX160_GYR_CONF_ODR_1600HZ               (0x0C)
#define BMX160_GYR_CONF_ODR_3200HZ               (0x0D)
/* Gyro bandwidth */
#define BMX160_GYR_CONF_BWP_OSR4             (0x00 << 4)
#define BMX160_GYR_CONF_BWP_OSR2             (0x01 << 4)
#define BMX160_GYR_CONF_BWP_NORMAL           (0x02 << 4)



/* Gyro Range */
#define BMX160_GYR_RANGE_2000_DPS           (0x00)
#define BMX160_GYR_RANGE_1000_DPS           (0x01)
#define BMX160_GYR_RANGE_500_DPS            (0x02)
#define BMX160_GYR_RANGE_250_DPS            (0x03)
#define BMX160_GYR_RANGE_125_DPS            (0x04)

/* Mag Output Data Rate */
#define BMX160_MAG_CONF_ODR_0_78HZ                (0x01)
#define BMX160_MAG_CONF_ODR_1_56HZ                (0x02)
#define BMX160_MAG_CONF_ODR_3_12HZ                (0x03)
#define BMX160_MAG_CONF_ODR_6_25HZ                (0x04)
#define BMX160_MAG_CONF_ODR_12_5HZ                (0x05)
#define BMX160_MAG_CONF_ODR_25HZ                  (0x06)
#define BMX160_MAG_CONF_ODR_50HZ                  (0x07)
#define BMX160_MAG_CONF_ODR_100HZ                 (0x08)
#define BMX160_MAG_CONF_ODR_200HZ                 (0x09)
#define BMX160_MAG_CONF_ODR_400HZ                 (0x0A)
#define BMX160_MAG_CONF_ODR_800HZ                 (0x0B)


#define BMX160_MAG_IF_0_CFG_MANUAL_EN (1 << 7)
#define BMX160_MAG_IF_0_CFG_OFFSET_0 (0 << 5)
#define BMX160_MAG_IF_0_CFG_RD_BURST_1 (0x0 << 0)
#define BMX160_MAG_IF_0_CFG_RD_BURST_2 (0x1 << 0)
#define BMX160_MAG_IF_0_CFG_RD_BURST_6 (0x2 << 0)
#define BMX160_MAG_IF_0_CFG_RD_BURST_8 (0x3 << 0)


#define BMX160_CMD_START_FOC                   (0x03)
#define BMX160_CMD_PROG_NVM                    (0xA0)
#define BMX160_CMD_FIFO_FLUSH                  (0xB0)
#define BMX160_CMD_INT_RESET                   (0xB1)
#define BMX160_CMD_SOFTRESET                   (0xB6)
#define BMX160_CMD_STEP_CNT_CLR                (0xB2)

#define BMX160_CMD_PMU_MODE_ACC_SUSPEND        (0x10 | (0x00 << 2) | 0x0)
#define BMX160_CMD_PMU_MODE_ACC_NORMAL         (0x10 | (0x00 << 2) | 0x1)
#define BMX160_CMD_PMU_MODE_ACC_LOW_POWER      (0x10 | (0x00 << 2) | 0x2)
// 0x2 mode for GYR is "Reserved" according to manual
#define BMX160_CMD_PMU_MODE_GYR_SUSPEND        (0x10 | (0x01 << 2) | 0x0)
#define BMX160_CMD_PMU_MODE_GYR_NORMAL         (0x10 | (0x01 << 2) | 0x1)
#define BMX160_CMD_PMU_MODE_GYR_FAST_STARTUP   (0x10 | (0x01 << 2) | 0x3)

#define BMX160_CMD_PMU_MODE_MAG_SUSPEND        (0x10 | (0x02 << 2) | 0x0)
#define BMX160_CMD_PMU_MODE_MAG_NORMAL         (0x10 | (0x02 << 2) | 0x1)
#define BMX160_CMD_PMU_MODE_MAG_LOW_POWER      (0x10 | (0x02 << 2) | 0x2)




// unit="m/s^2",
#define BMX160_SI_UNIT_FACT_ACC_2G (2.0 * 9.81 / 32768.0)
// unit="dps",
#define BMX160_SI_UNIT_FACT_GYR_2000DPS (8.0 * 250.0 / 32768.0)
// unit="T",
#define BMX160_SI_UNIT_FACT_MAG (6.67 * 4915.0 / 32768.0)


#endif
