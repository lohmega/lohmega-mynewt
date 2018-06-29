#ifndef __SENSOR_SI1133_H__
#define __SENSOR_SI1133_H__

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/
#define X_ORDER_MASK 0x0070
#define Y_ORDER_MASK 0x0007
#define SIGN_MASK 0x0080
#define get_x_order(m) ((m & X_ORDER_MASK) >> 4)
#define get_y_order(m) ((m & Y_ORDER_MASK))
#define get_sign(m) ((m & SIGN_MASK) >> 7)

#define UV_INPUT_FRACTION 15
#define UV_OUTPUT_FRACTION 12
#define UV_NUMCOEFF 2

#define ADC_THRESHOLD 16000
#define INPUT_FRACTION_HIGH 7
#define INPUT_FRACTION_LOW 15
#define LUX_OUTPUT_FRACTION 12
#define NUMCOEFF_LOW 9
#define NUMCOEFF_HIGH 4
/***************************************************************************/

#define I2C_ADDRESS 0b1010101
#define SI1133_OK 0x0000
#define SI1133_ERROR_I2C_TRANSACTION_FAILED 0x0001
#define SI1133_ERROR_SLEEP_FAILED 0x0002

/***************************************************************************/
struct si1133_cfg{
    uint8_t int_enable;
    sensor_type_t mask;
};
/***************************************************************************/
struct si1133 {
    struct os_dev dev;
    struct sensor sensor;
    struct os_mutex *i2c_mutex;
    struct si1133_cfg cfg;
    os_time_t last_read_time;
};

/***************************************************************************/
typedef struct {
    uint8_t irq_status;
    int32_t ch0;
    int32_t ch1;
    int32_t ch2;
    int32_t ch3;
} SI1133_Samples_TypeDef;

/***************************************************************************/
typedef struct {
    int16_t info;
    uint16_t mag;
} SI1133_Coeff_TypeDef;

/***************************************************************************/
typedef struct {
    SI1133_Coeff_TypeDef coeff_high[4];
    SI1133_Coeff_TypeDef coeff_low[9];
} SI1133_LuxCoeff_TypeDef;
/***************************************************************************/
#define SI1133_REG_PART_ID 0x00
#define SI1133_REG_HW_ID 0x01
#define SI1133_REG_REV_ID 0x02
#define SI1133_REG_HOSTIN0 0x0A
#define SI1133_REG_COMMAND 0x0B
#define SI1133_REG_IRQ_ENABLE 0x0F
#define SI1133_REG_RESPONSE1 0x10
#define SI1133_REG_RESPONSE0 0x11
#define SI1133_REG_IRQ_STATUS 0x12
#define SI1133_REG_HOSTOUT0 0x13
#define SI1133_REG_HOSTOUT1 0x14
#define SI1133_REG_HOSTOUT2 0x15
#define SI1133_REG_HOSTOUT3 0x16
#define SI1133_REG_HOSTOUT4 0x17
#define SI1133_REG_HOSTOUT5 0x18
#define SI1133_REG_HOSTOUT6 0x19
#define SI1133_REG_HOSTOUT7 0x1A
#define SI1133_REG_HOSTOUT8 0x1B
#define SI1133_REG_HOSTOUT9 0x1C
#define SI1133_REG_HOSTOUT10 0x1D
#define SI1133_REG_HOSTOUT11 0x1E
#define SI1133_REG_HOSTOUT12 0x1F
#define SI1133_REG_HOSTOUT13 0x20
#define SI1133_REG_HOSTOUT14 0x21
#define SI1133_REG_HOSTOUT15 0x22
#define SI1133_REG_HOSTOUT16 0x23
#define SI1133_REG_HOSTOUT17 0x24
#define SI1133_REG_HOSTOUT18 0x25
#define SI1133_REG_HOSTOUT19 0x26
#define SI1133_REG_HOSTOUT20 0x27
#define SI1133_REG_HOSTOUT21 0x28
#define SI1133_REG_HOSTOUT22 0x29
#define SI1133_REG_HOSTOUT23 0x2A
#define SI1133_REG_HOSTOUT24 0x2B
#define SI1133_REG_HOSTOUT25 0x2C
/***************************************************************************/
#define SI1133_PARAM_I2C_ADDR 0x00
#define SI1133_PARAM_CH_LIST 0x01
#define SI1133_PARAM_ADCCONFIG0 0x02
#define SI1133_PARAM_ADCSENS0 0x03
#define SI1133_PARAM_ADCPOST0 0x04
#define SI1133_PARAM_MEASCONFIG0 0x05
#define SI1133_PARAM_ADCCONFIG1 0x06
#define SI1133_PARAM_ADCSENS1 0x07
#define SI1133_PARAM_ADCPOST1 0x08
#define SI1133_PARAM_MEASCONFIG1 0x09
#define SI1133_PARAM_ADCCONFIG2 0x0A
#define SI1133_PARAM_ADCSENS2 0x0B
#define SI1133_PARAM_ADCPOST2 0x0C
#define SI1133_PARAM_MEASCONFIG2 0x0D
#define SI1133_PARAM_ADCCONFIG3 0x0E
#define SI1133_PARAM_ADCSENS3 0x0F
#define SI1133_PARAM_ADCPOST3 0x10
#define SI1133_PARAM_MEASCONFIG3 0x11
#define SI1133_PARAM_ADCCONFIG4 0x12
#define SI1133_PARAM_ADCSENS4 0x13
#define SI1133_PARAM_ADCPOST4 0x14
#define SI1133_PARAM_MEASCONFIG4 0x15
#define SI1133_PARAM_ADCCONFIG5 0x16
#define SI1133_PARAM_ADCSENS5 0x17
#define SI1133_PARAM_ADCPOST5 0x18
#define SI1133_PARAM_MEASCONFIG5 0x19
#define SI1133_PARAM_MEASRATE_H 0x1A
#define SI1133_PARAM_MEASRATE_L 0x1B
#define SI1133_PARAM_MEASCOUNT0 0x1C
#define SI1133_PARAM_MEASCOUNT1 0x1D
#define SI1133_PARAM_MEASCOUNT2 0x1E
#define SI1133_PARAM_THRESHOLD0_H 0x25
#define SI1133_PARAM_THRESHOLD0_L 0x26
#define SI1133_PARAM_THRESHOLD1_H 0x27
#define SI1133_PARAM_THRESHOLD1_L 0x28
#define SI1133_PARAM_THRESHOLD2_H 0x29
#define SI1133_PARAM_THRESHOLD2_L 0x2A
#define SI1133_PARAM_BURST 0x2B
/***************************************************************************/
#define SI1133_CMD_RESET_CMD_CTR 0x00
#define SI1133_CMD_RESET 0x01
#define SI1133_CMD_NEW_ADDR 0x02
#define SI1133_CMD_FORCE_CH 0x11
#define SI1133_CMD_PAUSE_CH 0x12
#define SI1133_CMD_START 0x13
#define SI1133_CMD_PARAM_SET 0x80
#define SI1133_CMD_PARAM_QUERY 0x40
/***************************************************************************/
#define SI1133_RSP0_CHIPSTAT_MASK 0xE0
#define SI1133_RSP0_COUNTER_MASK 0x1F
#define SI1133_RSP0_SLEEP 0x20
/***************************************************************************/


uint32_t SI1133_registerWrite(uint8_t reg, uint8_t data);
uint32_t SI1133_registerRead(uint8_t reg, uint8_t *data);
uint32_t SI1133_registerBlockRead(uint8_t reg, uint8_t length, uint8_t *data);
uint32_t SI1133_registerBlockWrite(uint8_t reg, uint8_t length, uint8_t *data);
uint32_t SI1133_reset(void);
uint32_t SI1133_resetCmdCtr(void);
uint32_t SI1133_measurementForce(void);
uint32_t SI1133_measurementPause(void);
uint32_t SI1133_measurementStart(void);
uint32_t SI1133_waitUntilSleep(void);
uint32_t SI1133_paramSet(uint8_t address, uint8_t value);
uint32_t SI1133_paramRead(uint8_t address);
uint32_t SI1133_init(void);
uint32_t SI1133_deInit(void);
uint32_t SI1133_measurementGet(SI1133_Samples_TypeDef *samples);
int32_t SI1133_getUv(int32_t uv, SI1133_Coeff_TypeDef *uk);
int32_t SI1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir, SI1133_LuxCoeff_TypeDef *lk);
uint32_t SI1133_measureLuxUvif(float *lux, float *uvi);
uint32_t SI1133_measureLuxUvi(int32_t *lux, int32_t *uvi);
uint32_t SI1133_getHardwareID(uint8_t *hardwareID);
uint32_t SI1133_getMeasurementf(float *lux, float *uvi);
uint32_t SI1133_getMeasurement(int32_t *lux, int32_t *uvi);
uint32_t SI1133_getIrqStatus(uint8_t *irqStatus);
uint32_t SI1133_enableIrq0(bool enable);
uint32_t si1133_config(void);
int si1133_init(struct os_dev *dev, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_HTS221_H__ */