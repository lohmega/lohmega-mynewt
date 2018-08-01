

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include "defs/error.h"
#include "os/os.h"
#include "os/os_mutex.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/light.h"
#include "si1133/SI1133.h"
#include "log/log.h"


#include <stats/stats.h>


STATS_SECT_START(si1133_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(si1133_stats) g_si1133_stats;

/* Define the stats section and records */
STATS_NAME_START(si1133_stats)
    STATS_NAME(si1133_stats, read_errors)
    STATS_NAME(si1133_stats, write_errors)
    STATS_NAME(si1133_stats, mutex_errors)
STATS_NAME_END(si1133_stats)

#define LOG_MODULE_SI1133    (221)
#define SI1133_INFO(...)     LOG_INFO(&_log, LOG_MODULE_SI1133, __VA_ARGS__)
#define SI1133_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_SI1133, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int si1133_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);

static int si1133_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_si1133_sensor_driver = {
    si1133_sensor_read,
    si1133_sensor_get_config
};


static si1133_LuxCoeff_TypeDef lk = {

    {{0, 209}, {1665, 93}, {2064, 65}, {-2671, 234}},
    {{0, 0},
     {1921, 29053},
     {-1022, 36363},
     {2320, 20789},
     {-367, 57909},
     {-1774, 38240},
     {-608, 46775},
     {-1503, 51831},
     {-1886, 58928}}};

static volatile int g_task1_loops;
static si1133_Coeff_TypeDef uk[2] = {{1281, 30902}, {-638, 46301}};
static int32_t si1133_calcPolyInner(int32_t input, int8_t fraction,
                                    uint16_t mag, int8_t shift);      
static int32_t si1133_calcEvalPoly(int32_t x, int32_t y,
                                   uint8_t input_fraction,
                                   uint8_t output_fraction, uint8_t num_coeff,
                                   si1133_Coeff_TypeDef *kp);





/**
 * @brief Read hardwareID from si1133
 * 
 * @param hardwareID
 * 
 * @return 0x0000 if ok
 */
uint32_t
si1133_getHardwareID(struct si1133 *dev, uint8_t *hardwareID)
{
    uint32_t rc;
    rc = si1133_registerRead(dev, SI1133_REG_PART_ID, hardwareID);
    return rc;
}


/**
 * @brief Write data to reg (register)
 * @param reg
 * @param data
 * @return 0x0000 if ok
 */
uint32_t
si1133_registerWrite(struct si1133 *dev, uint8_t reg, uint8_t data)
{
    
    int rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;
    uint8_t out[2] = {reg, data};

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr, 
        .len = 2,
        .buffer = out
    };

    if (dev->i2c_mutex) {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            SI1133_ERR("Mutex error=%d\n", err);
            return err;
        }
    }

    rc = hal_i2c_master_write(1, &data_struct, OS_TICKS_PER_SEC / 10, 0);

    if (rc) {
        SI1133_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                     itf->si_addr, reg, data);
        STATS_INC(g_si1133_stats, write_errors);
    }

    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }

    return rc;
}

/**
 * @biref Read from reg, write to *data
 * @param reg, register to read to
 * @param data, array to put content in
 * @return  0x0000 if ok
 */
uint32_t
si1133_registerRead(struct si1133 *dev, uint8_t reg, uint8_t *data)
{
    uint32_t rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };



    if (dev->i2c_mutex) {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK) {
            SI1133_ERR("Mutex error=%d\n", err);
            STATS_INC(g_si1133_stats, mutex_errors);
            return err;
        }
    }
 
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);

    if (rc) {
        SI1133_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_si1133_stats, write_errors);
        return rc;
    }


    data_struct.buffer = data;

    rc = hal_i2c_master_read(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        SI1133_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
        STATS_INC(g_si1133_stats, write_errors);
        return rc;
    }

    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }

    return rc;
}

/**
 * @brief Wait until si1133 is knwon ro be in sleep state
 * 
 * @return 0x0000 if ok
 */
uint32_t
si1133_waitUntilSleep(struct si1133 *dev)
{
    uint32_t ret;
    uint8_t response;
    uint8_t count = 0;
    uint32_t retval;

    retval = SI1133_OK;

    /* This loops until the si1133 is known to be in its sleep state  */
    /* or if an i2c error occurs                                      */
    while (count < 5) {
        ret = si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_CHIPSTAT_MASK) == SI1133_RSP0_SLEEP) {
            break;
        }

        if (ret != SI1133_OK) {
            retval = SI1133_ERROR_SLEEP_FAILED;
            SI1133_ERR("Failed to sleep \n");
            STATS_INC(g_si1133_stats, write_errors);
            break;
        }

        count++;
    }

    return retval;
}

uint32_t
si1133_registerBlockRead(struct si1133 *dev, uint8_t reg, uint8_t length,
    uint8_t *data)
{
    uint32_t rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;

    rc = SI1133_OK;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    if(dev->i2c_mutex) {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK) {
            SI1133_ERR("Mutex error=%d\n", err);
            return err;
        }
    }

    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    data_struct.len = length;
    data_struct.buffer = data;

    rc = hal_i2c_master_read(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }

    return rc;
}

/**
 * @param reg,
 * @param length,
 * @param data,
 */
uint32_t
si1133_registerBlockWrite(struct si1133 *dev, uint8_t reg, uint8_t length,
    uint8_t *data)
{

    uint8_t i2c_write_data[length];
    uint32_t rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;

    i2c_write_data[0] = reg;

    uint8_t i;
    for (i = 0; i < length; i++) {
        i2c_write_data[i+1] = data[i];
    }

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = length+1,
        .buffer = i2c_write_data
    };

    if (dev->i2c_mutex) {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK) {
            SI1133_ERR("Mutex error=%d\n", err);
            STATS_INC(g_si1133_stats, mutex_errors);
            return err;
        }
    }

    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        SI1133_ERR("Failed block write\n");
        STATS_INC(g_si1133_stats, write_errors);
        return rc;
    }

    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }

    return rc;
}


/**
 * 
 * @param address, The parameter address
 * @param val, The byte to be written
 * 
 * @return, Returns 0x0000 for OK
 */
uint32_t
si1133_paramSet(struct si1133 *dev, uint8_t address, uint8_t value)
{

    uint32_t retval = 1;
    uint8_t buffer[2];
    uint8_t response_stored;
    uint8_t response;
    uint8_t count;

    int i = 0;

    while (retval != SI1133_OK && i <= 2){
        retval = si1133_waitUntilSleep(dev);
        i ++;
    }    

    if (retval != SI1133_OK) {
        SI1133_ERR("Failed to wait until sleep \n");
        STATS_INC(g_si1133_stats, write_errors);
        return retval;
    }


    si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response_stored);
    response_stored &= SI1133_RSP0_COUNTER_MASK;

    buffer[0] = value;
    buffer[1] = 0x80 + (address & 0x3F);

    retval = si1133_registerBlockWrite(dev, SI1133_REG_HOSTIN0, 2,
        (uint8_t *)buffer);
    if (retval != SI1133_OK) {
        SI1133_ERR("Failed to block write\n");
        STATS_INC(g_si1133_stats, write_errors);
        return retval;
    }

    /* Wait for command to finish */
    count = 0;
    /* Expect a change in the response register */
    while (count < 5) {

        retval = si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_COUNTER_MASK) != response_stored) {
            break;
        } else {
            if (retval != SI1133_OK) {
                return retval;
            }
        }

        count++;
    }

    return SI1133_OK;
}

/**
 * @brief reset si1133
 */
uint32_t
si1133_reset(struct si1133 *dev)
{
    uint32_t rc;
    os_time_delay(5);
    rc = si1133_registerWrite(dev, SI1133_REG_COMMAND, SI1133_CMD_RESET);
    os_time_delay(5);

    if(rc){
        SI1133_ERR("Failed to reset for si1133\n");
        STATS_INC(g_si1133_stats, write_errors);
    }
    return rc;
}

int32_t
si1133_calcPolyInner(int32_t input, int8_t fraction, uint16_t mag,
                             int8_t shift)
{

    int32_t value;

    if (shift < 0) {
        value = ((input << fraction) / mag) >> -shift;
    } else {
        value = ((input << fraction) / mag) << shift;
    }

    return value;
}

int32_t
si1133_calcEvalPoly(int32_t x, int32_t y, uint8_t input_fraction,
                            uint8_t output_fraction, uint8_t num_coeff,
                            si1133_Coeff_TypeDef *kp)
{

    uint8_t info, x_order, y_order, counter;
    int8_t sign, shift;
    uint16_t mag;
    int32_t output = 0, x1, x2, y1, y2;

    for (counter = 0; counter < num_coeff; counter++) {

        info = kp->info;
        x_order = get_x_order(info);
        y_order = get_y_order(info);

        shift = ((uint16_t)kp->info & 0xff00) >> 8;
        shift ^= 0x00ff;
        shift += 1;
        shift = -shift;

        mag = kp->mag;

        if (get_sign(info)) {
            sign = -1;
        } else {
            sign = 1;
        }

        if ((x_order == 0) && (y_order == 0)) {
            output += sign * mag << output_fraction;
        } else {
            if (x_order > 0) {
                x1 = si1133_calcPolyInner(x, input_fraction, mag, shift);
                if (x_order > 1) {
                    x2 = si1133_calcPolyInner(x, input_fraction, mag, shift);
                } else {
                    x2 = 1;
                }
            } else {
                x1 = 1;
                x2 = 1;
            }

            if (y_order > 0) {
                y1 = si1133_calcPolyInner(y, input_fraction, mag, shift);
                if (y_order > 1) {
                    y2 = si1133_calcPolyInner(y, input_fraction, mag, shift);
                } else {
                    y2 = 1;
                }
            } else {
                y1 = 1;
                y2 = 1;
            }

            output += sign * x1 * x2 * y1 * y2;
        }

        kp++;
    }

    if (output < 0) {
        output = -output;
    }

    return output;
}

uint32_t
si1133_deInit(struct si1133 *dev)
{

    uint32_t retval;

    retval = si1133_paramSet(dev, SI1133_PARAM_CH_LIST, 0x3f);
    retval += si1133_measurementPause(dev);
    retval += si1133_waitUntilSleep(dev);

    return retval;
}


uint32_t
si1133_measurementGet(struct si1133 *dev, si1133_Samples_TypeDef *samples)
{

    uint8_t buffer[13];
    uint32_t retval;

    retval = si1133_registerBlockRead(dev, SI1133_REG_IRQ_STATUS, 13, buffer);

    samples->irq_status = buffer[0];

    samples->ch0 = buffer[1] << 16;
    samples->ch0 |= buffer[2] << 8;
    samples->ch0 |= buffer[3];
    if (samples->ch0 & 0x800000) {
        samples->ch0 |= 0xFF000000;
    }

    samples->ch1 = buffer[4] << 16;
    samples->ch1 |= buffer[5] << 8;
    samples->ch1 |= buffer[6];
    if (samples->ch1 & 0x800000) {
        samples->ch1 |= 0xFF000000;
    }

    samples->ch2 = buffer[7] << 16;
    samples->ch2 |= buffer[8] << 8;
    samples->ch2 |= buffer[9];
    if (samples->ch2 & 0x800000) {
        samples->ch2 |= 0xFF000000;
    }

    samples->ch3 = buffer[10] << 16;
    samples->ch3 |= buffer[11] << 8;
    samples->ch3 |= buffer[12];
    if (samples->ch3 & 0x800000) {
        samples->ch3 |= 0xFF000000;
    }

    return retval;
}

int32_t
si1133_getUv(int32_t uv, si1133_Coeff_TypeDef *uk)
{

    int32_t uvi;

    uvi = si1133_calcEvalPoly(0, uv, UV_INPUT_FRACTION, UV_OUTPUT_FRACTION,
                              UV_NUMCOEFF, uk);

    return uvi;
}

int32_t
si1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir,
                      si1133_LuxCoeff_TypeDef *lk)
{

    int32_t lux;

    if ((vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD)) {
        lux = si1133_calcEvalPoly(vis_high, ir, INPUT_FRACTION_HIGH,
                                  LUX_OUTPUT_FRACTION, NUMCOEFF_HIGH,
                                  &(lk->coeff_high[0]));
    } else {
        lux = si1133_calcEvalPoly(vis_low, ir, INPUT_FRACTION_LOW,
                                  LUX_OUTPUT_FRACTION, NUMCOEFF_LOW,
                                  &(lk->coeff_low[0]));
    }

    return lux;
}

uint32_t
si1133_measureLuxUvi(struct si1133 *dev, int32_t *lux, int32_t *uvi)
{

    si1133_Samples_TypeDef samples;
    uint32_t retval;
    uint8_t response;

    /* Force measurement */
    retval = si1133_measurementForce(dev);

    /* Go to sleep while the sensor does the conversion */
    os_time_delay(20);

    /* Check if the measurement finished, if not then wait */
    retval += si1133_registerRead(dev, SI1133_REG_IRQ_STATUS, &response);
    while (response != 0x0F) {
        os_time_delay(1);
        retval += si1133_registerRead(dev, SI1133_REG_IRQ_STATUS, &response);
    }

    /* Get the results */
    si1133_measurementGet(dev, &samples);
    
    /* Convert the readings to lux */
    int64_t v = si1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = (v*1000) >> LUX_OUTPUT_FRACTION;

    /* Convert the readings to UV index */
    v = si1133_getUv(samples.ch0, uk);
    *uvi = (v*1000) >> UV_OUTPUT_FRACTION;

    return retval;
}

/**
 * @param command
 */
static uint32_t
si1133_sendCmd(struct si1133 *dev, uint8_t command)
{

    uint8_t response;
    uint8_t response_stored;
    uint8_t count = 0;
    uint32_t ret;

    /* Get the response register contents */
    ret = si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response_stored);
    if (ret != SI1133_OK) {
        return ret;
    }

    response_stored = response_stored & SI1133_RSP0_COUNTER_MASK;

    /* Double-check the response register is consistent */
    while (count < 5) {
        ret = si1133_waitUntilSleep(dev);
        if (ret != SI1133_OK) {
            return ret;
        }
        /* Skip if the command is RESET COMMAND COUNTER */
        if (command == SI1133_CMD_RESET_CMD_CTR) {
            break;
        }

        ret = si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response);

        if ((response & SI1133_RSP0_COUNTER_MASK) == response_stored) {
            break;
        } else {
            if (ret != SI1133_OK) {
                return ret;
            }
            else {
                response_stored = response & SI1133_RSP0_COUNTER_MASK;
            }
        }

        count++;
    }

    /* Send the command */
    ret = si1133_registerWrite(dev, SI1133_REG_COMMAND, command);
    if (ret != SI1133_OK) {
        return ret;
    }

    count = 0;
    /* Expect a change in the response register */
    while (count < 5) {
        /* Skip if the command is RESET COMMAND COUNTER */
        if (command == SI1133_CMD_RESET_CMD_CTR) {
            break;
        }

        ret = si1133_registerRead(dev, SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_COUNTER_MASK) != response_stored) {
            break;
        } else {
            if (ret != SI1133_OK) {
                return ret;
            }
        }

        count++;
    }

    return SI1133_OK;
}

uint32_t si1133_measurementForce(struct si1133 *dev)
{
    return si1133_sendCmd(dev, SI1133_CMD_FORCE_CH);
}

uint32_t si1133_resetCmdCtr(struct si1133 *dev)
{
    return si1133_sendCmd(dev, SI1133_CMD_RESET_CMD_CTR);
}

uint32_t si1133_measurementStart(struct si1133 *dev)
{
    return si1133_sendCmd(dev, SI1133_CMD_START);
}

uint32_t si1133_paramRead(struct si1133 *dev, uint8_t address)
{

    uint8_t retval;
    uint8_t cmd;

    cmd = 0x40 + (address & 0x3F);

    retval = si1133_sendCmd(dev, cmd);
    if (retval != SI1133_OK) {
        return retval;
    }

    si1133_registerRead(dev, SI1133_REG_RESPONSE1, &retval);

    return retval;
}

uint32_t
si1133_getMeasurementf(struct si1133 *dev, float *lux, float *uvi)
{

    si1133_Samples_TypeDef samples;
    uint32_t retval;

    /* Get the results */
    retval = si1133_measurementGet(dev, &samples);

    /* Convert the readings to lux */
    *lux = (float)si1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    *uvi = (float)si1133_getUv(samples.ch0, uk);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return retval;
}

uint32_t
si1133_getMeasurement(struct si1133 *dev, int32_t *lux, int32_t *uvi)
{

    si1133_Samples_TypeDef samples;
    uint32_t retval;

    /* Get the results */
    retval = si1133_measurementGet(dev, &samples);

    /* Convert the readings to lux */
    int64_t v = si1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = (int32_t)((v*1000) >> LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    v = si1133_getUv(samples.ch0, uk);
    *uvi = (int32_t)((v*1000) >> UV_OUTPUT_FRACTION);
    
    return retval;
}

uint32_t
si1133_getIrqStatus(struct si1133 *dev, uint8_t *irqStatus)
{

    uint32_t retval;

    /* Read the IRQ status register */
    retval = si1133_registerRead(dev, SI1133_REG_IRQ_STATUS, irqStatus);

    return retval;
}

uint32_t
si1133_enableIrq0(struct si1133 *dev, bool enable)
{

    uint32_t retval;

    if (enable) {
        retval = si1133_registerWrite(dev, SI1133_REG_IRQ_ENABLE, 0x0F);
    } else {
        retval = si1133_registerWrite(dev, SI1133_REG_IRQ_ENABLE, 0);
    }

    //APP_ERROR_CHECK(retval);

    return retval;
}


/**
 * @brief get a measurement from SI1133
 * @param lux, ptr to where to put lux measurement
 * @param uvi, ptr to where to put uv measurement
 */
uint32_t
si1133_measureLuxUvif(struct si1133 *dev, float *lux, float *uvi)
{

    si1133_Samples_TypeDef samples;
    uint32_t retval;
    uint8_t response;

    /* Force measurement */
    retval = si1133_measurementForce(dev);

    /* Go to sleep while the sensor does the conversion */
    os_time_delay(20);

    /* Check if the measurement finished, if not then wait */
    retval += si1133_registerRead(dev, SI1133_REG_IRQ_STATUS, &response);
    while (response != 0x0F) {
        os_time_delay(10);
        retval += si1133_registerRead(dev, SI1133_REG_IRQ_STATUS, &response);
    }

    /* Get the results */
    si1133_measurementGet(dev, &samples);
   
    /* Convert the readings to lux */
    *lux = (float)si1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    *uvi = (float)si1133_getUv(samples.ch0, uk);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return retval;
}

uint32_t
si1133_measurementPause(struct si1133 *dev)
{
    return si1133_sendCmd(dev, SI1133_CMD_PAUSE_CH);
}

uint32_t
si1133_config(struct si1133 *si1, struct si1133_cfg *cfg)
{

    int rc;

    /*Allow some time for the sensor to power up*/
    os_time_delay(10);
    
    rc = si1133_reset(si1);

    os_time_delay(10);
    
    rc += si1133_paramSet(si1, SI1133_PARAM_CH_LIST, 0x0f);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCCONFIG0, 0x78);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCSENS0, 0x11);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCPOST0, 0x40);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCCONFIG1, 0x4d);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCSENS1, 0x91);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCPOST1, 0x40);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCCONFIG2, 0x41);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCSENS2, 0x91);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCPOST2, 0x50);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCCONFIG3, 0x4d);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCSENS3, 0x17);
    rc += si1133_paramSet(si1, SI1133_PARAM_ADCPOST3, 0x40);

    rc += si1133_registerWrite(si1, SI1133_REG_IRQ_ENABLE, 0x0f);
    si1->cfg.int_enable = cfg->int_enable;

    uint8_t irq;
    rc += si1133_getIrqStatus(si1, &irq);

    rc += sensor_set_type_mask(&(si1->sensor), cfg->mask);
    si1->cfg.mask = cfg->mask;

    if (rc) {
        SI1133_ERR("si1133 config fail");
    }
    

    return rc;
} 
 

int
si1133_init(struct os_dev *dev, void *arg)
{    
    struct si1133 *si1;
    struct sensor *sensor;
    int rc;

    si1 = (struct si1133 *) dev;

    si1->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &si1->sensor;

    rc = sensor_init(sensor, dev);
    if(rc){
        return rc;
    }

    /* Add lux/uv driver*/
    rc = sensor_set_driver(sensor, SENSOR_TYPE_LIGHT, 
            (struct sensor_driver *) &g_si1133_sensor_driver);
    if(rc){
        return rc;
    }
    
    rc = sensor_set_interface(sensor, arg);
    if(rc){
        return rc;
    }

    rc = sensor_mgr_register(sensor);

    return rc;
}

static int
si1133_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    int32_t lux, uvi;
    struct si1133 *si1;
    struct sensor_light_data sld;

    if (!(type & SENSOR_TYPE_LIGHT)){
        return SYS_EINVAL;
    }

    si1 = (struct si1133 *) SENSOR_GET_DEVICE(sensor);

    if (type & (SENSOR_TYPE_LIGHT)){
        rc = si1133_measureLuxUvi(si1, &lux, &uvi);

        if(rc) {
            return rc;
        }


        sld.sld_lux = lux;
        sld.sld_ir = (uint16_t)uvi;

        rc = data_func(sensor, data_arg, &sld, SENSOR_TYPE_LIGHT);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

static int
si1133_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    if (!(type & (SENSOR_TYPE_LIGHT))) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_INT32;

    return 0;
}