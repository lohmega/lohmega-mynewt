

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

///* only used for debugging*/
//#include <inttypes.h> 
//#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
//#define BYTE_TO_BINARY(byte)  \
//  (byte & 0x80 ? '1' : '0'), \
//  (byte & 0x40 ? '1' : '0'), \
//  (byte & 0x20 ? '1' : '0'), \
//  (byte & 0x10 ? '1' : '0'), \
//  (byte & 0x08 ? '1' : '0'), \
//  (byte & 0x04 ? '1' : '0'), \
//  (byte & 0x02 ? '1' : '0'), \
//  (byte & 0x01 ? '1' : '0') 
///* only used for debugging*/


#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif
#include "console/console.h"
#include "hal/hal_i2c.h"

// TODO do the header file propertly
#include "sensor/SI1133.h"


/***************************************************************************/
static SI1133_LuxCoeff_TypeDef lk = {

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

/***************************************************************************/
static volatile int g_task1_loops;
static SI1133_Coeff_TypeDef uk[2] = {{1281, 30902}, {-638, 46301}};
static int32_t SI1133_calcPolyInner(int32_t input, int8_t fraction,
                                    uint16_t mag, int8_t shift);      
static int32_t SI1133_calcEvalPoly(int32_t x, int32_t y, uint8_t input_fraction,
                                   uint8_t output_fraction, uint8_t num_coeff,
                                   SI1133_Coeff_TypeDef *kp);





/**
 * @brief Read hardwareID from SI1133
 * 
 * @param hardwareID
 * 
 * @return 0x0000 if ok
 */
uint32_t SI1133_getHardwareID(uint8_t *hardwareID){
    uint32_t rc;
    rc = SI1133_registerRead(SI1133_REG_PART_ID, hardwareID);
    return rc;
}


/**
 * @brief Write data to reg (register)
 * @param reg
 * @param data
 * @return 0x0000 if ok
 */
uint32_t SI1133_registerWrite(uint8_t reg, uint8_t data){
    int rc;

    uint8_t out[2] = {reg, data};

    struct hal_i2c_master_data data_struct = {
        .address = I2C_ADDRESS, 
        .len = 2,
        .buffer = out
    };

    rc = hal_i2c_master_write(1, &data_struct, OS_TICKS_PER_SEC / 10, 0);

    if (rc){
        console_printf("faild to write");
        console_printf("\n");
    }


    return rc;
}

/**
 * @biref Read from reg, write to *data
 * @param reg, register to write to
 * @param data, array to put content in
 * @return  0x0000 if ok
 */
uint32_t SI1133_registerRead(uint8_t reg, uint8_t *data)
{
    uint32_t rc;

    struct hal_i2c_master_data data_struct = {
        .address = I2C_ADDRESS,  // i2c addres, TODO write as a argument to function
        .len = 1,
        .buffer = &reg
    };

    rc = hal_i2c_master_write(1, &data_struct, OS_TICKS_PER_SEC / 10, 0); // TODO itf->si_num

    if (rc){
        console_printf("faild to write (for reading)");
        console_printf("\n");
        return rc;
    }


    data_struct.buffer = data;

    rc = hal_i2c_master_read(1, &data_struct, OS_TICKS_PER_SEC / 10, 1); // TODO itf->si_num

    if (rc){
        console_printf("faild to read");
        console_printf("\n");
        return rc;
    }

    return rc;
}

/**
 * @brief Wait until SI1133 is knwon ro be in sleep state
 * 
 * @return 0x0000 if ok
 */
uint32_t SI1133_waitUntilSleep(void){
    uint32_t ret;
    uint8_t response;
    uint8_t count = 0;
    uint32_t retval;

    retval = SI1133_OK;

    /* This loops until the SI1133 is known to be in its sleep state  */
    /* or if an i2c error occurs                                      */
    while (count < 5) {
        ret = SI1133_registerRead(SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_CHIPSTAT_MASK) == SI1133_RSP0_SLEEP) {
            break;
        }

        if (ret != SI1133_OK) {
            retval = SI1133_ERROR_SLEEP_FAILED;
            console_printf("Sleep faild \n");
            break;
        }

        count++;
    }

    return retval;
}

uint32_t SI1133_registerBlockRead(uint8_t reg, uint8_t length, uint8_t *data)
{

    uint32_t rc;

    rc = SI1133_OK;

    struct hal_i2c_master_data data_struct = {
        .address = I2C_ADDRESS,
        .len = 1,
        .buffer = &reg
    };

    rc = hal_i2c_master_write(1, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    data_struct.len = length;
    data_struct.buffer = data;

    rc = hal_i2c_master_read(1, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    return rc;
}

/**
 * @param reg,
 * @param length,
 * @param data,
 */
uint32_t SI1133_registerBlockWrite(uint8_t reg, uint8_t length, uint8_t *data){

    uint8_t i2c_write_data[length];
    uint32_t rc;

    i2c_write_data[0] = reg;

    uint8_t i;
    for(i = 0; i < length; i++){
        i2c_write_data[i+1] = data[i];
    }

    struct hal_i2c_master_data data_struct = {
        .address = I2C_ADDRESS,
        .len = length+1,
        .buffer = i2c_write_data
    };

    rc = hal_i2c_master_write(1, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc){
        console_printf("faild to write (block)");
        console_printf("\n");
        return rc;
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
uint32_t SI1133_paramSet(uint8_t address, uint8_t value){

    uint32_t retval = 1;
    uint8_t buffer[2];
    uint8_t response_stored;
    uint8_t response;
    uint8_t count;

    int i = 0;

    while (retval != SI1133_OK && i <= 2){
        retval = SI1133_waitUntilSleep();
        i ++;
    }    

    if (retval != SI1133_OK) {
        console_printf("Error wait to sleep, paramset \n");
        return retval;
    }


    SI1133_registerRead(SI1133_REG_RESPONSE0, &response_stored);
    response_stored &= SI1133_RSP0_COUNTER_MASK;

    buffer[0] = value;
    buffer[1] = 0x80 + (address & 0x3F);

    retval =
        SI1133_registerBlockWrite(SI1133_REG_HOSTIN0, 2, (uint8_t *)buffer);
    if (retval != SI1133_OK) {
        return retval;
    }

    /* Wait for command to finish */
    count = 0;
    /* Expect a change in the response register */
    while (count < 5) {

        retval = SI1133_registerRead(SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_COUNTER_MASK) != response_stored) {
            break;
        }
        else {
            if (retval != SI1133_OK) {
                return retval;
            }
        }

        count++;
    }

    return SI1133_OK;
}

/**
 * @brief reset SI1133
 */
uint32_t SI1133_reset(void){
    uint32_t rc;
    os_time_delay(5);
    rc = SI1133_registerWrite(SI1133_REG_COMMAND, SI1133_CMD_RESET);
    os_time_delay(5);

    if(rc){
        console_printf("faild to reset");
        console_printf("\n");
    }else{
        console_printf("succeeded to reset");
        console_printf("\n");
    }
    return rc;
}

int32_t SI1133_calcPolyInner(int32_t input, int8_t fraction, uint16_t mag,
                             int8_t shift)
{

    int32_t value;

    if (shift < 0) {
        value = ((input << fraction) / mag) >> -shift;
    }
    else {
        value = ((input << fraction) / mag) << shift;
    }

    return value;
}

int32_t SI1133_calcEvalPoly(int32_t x, int32_t y, uint8_t input_fraction,
                            uint8_t output_fraction, uint8_t num_coeff,
                            SI1133_Coeff_TypeDef *kp)
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
        }
        else {
            sign = 1;
        }

        if ((x_order == 0) && (y_order == 0)) {
            output += sign * mag << output_fraction;
        }
        else {
            if (x_order > 0) {
                x1 = SI1133_calcPolyInner(x, input_fraction, mag, shift);
                if (x_order > 1) {
                    x2 = SI1133_calcPolyInner(x, input_fraction, mag, shift);
                }
                else {
                    x2 = 1;
                }
            }
            else {
                x1 = 1;
                x2 = 1;
            }

            if (y_order > 0) {
                y1 = SI1133_calcPolyInner(y, input_fraction, mag, shift);
                if (y_order > 1) {
                    y2 = SI1133_calcPolyInner(y, input_fraction, mag, shift);
                }
                else {
                    y2 = 1;
                }
            }
            else {
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

uint32_t SI1133_deInit(void)
{

    uint32_t retval;

    retval = SI1133_paramSet(SI1133_PARAM_CH_LIST, 0x3f);
    retval += SI1133_measurementPause();
    retval += SI1133_waitUntilSleep();

    return retval;
}


uint32_t SI1133_measurementGet(SI1133_Samples_TypeDef *samples)
{

    uint8_t buffer[13];
    uint32_t retval;

    retval = SI1133_registerBlockRead(SI1133_REG_IRQ_STATUS, 13, buffer);

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

int32_t SI1133_getUv(int32_t uv, SI1133_Coeff_TypeDef *uk)
{

    int32_t uvi;

    uvi = SI1133_calcEvalPoly(0, uv, UV_INPUT_FRACTION, UV_OUTPUT_FRACTION,
                              UV_NUMCOEFF, uk);

    return uvi;
}

int32_t SI1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir,
                      SI1133_LuxCoeff_TypeDef *lk)
{

    int32_t lux;

    if ((vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD)) {
        lux = SI1133_calcEvalPoly(vis_high, ir, INPUT_FRACTION_HIGH,
                                  LUX_OUTPUT_FRACTION, NUMCOEFF_HIGH,
                                  &(lk->coeff_high[0]));
    }
    else {
        lux = SI1133_calcEvalPoly(vis_low, ir, INPUT_FRACTION_LOW,
                                  LUX_OUTPUT_FRACTION, NUMCOEFF_LOW,
                                  &(lk->coeff_low[0]));
    }

    return lux;
}

uint32_t SI1133_measureLuxUvi(int32_t *lux, int32_t *uvi)
{

    SI1133_Samples_TypeDef samples;
    uint32_t retval;
    uint8_t response;

    /* Force measurement */
    retval = SI1133_measurementForce();

    /* Go to sleep while the sensor does the conversion */
    os_time_delay(20);

    /* Check if the measurement finished, if not then wait */
    retval += SI1133_registerRead(SI1133_REG_IRQ_STATUS, &response);
    while (response != 0x0F) {
        os_time_delay(1);
        retval += SI1133_registerRead(SI1133_REG_IRQ_STATUS, &response);
    }

    /* Get the results */
    SI1133_measurementGet(&samples);
    //NRF_LOG_INFO("Raw vals %d, %d, %d, %d\r\n", samples.ch0, samples.ch1, samples.ch2, samples.ch3);
    /* Convert the readings to lux */
    int64_t v = SI1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = (v*1000) >> LUX_OUTPUT_FRACTION;

    /* Convert the readings to UV index */
    v = SI1133_getUv(samples.ch0, uk);
    *uvi = (v*1000) >> UV_OUTPUT_FRACTION;

    return retval;
}

/**
 * @param command
 */
static uint32_t SI1133_sendCmd(uint8_t command)
{

    uint8_t response;
    uint8_t response_stored;
    uint8_t count = 0;
    uint32_t ret;

    /* Get the response register contents */
    ret = SI1133_registerRead(SI1133_REG_RESPONSE0, &response_stored);
    if (ret != SI1133_OK) {
        return ret;
    }

    response_stored = response_stored & SI1133_RSP0_COUNTER_MASK;

    /* Double-check the response register is consistent */
    while (count < 5) {
        ret = SI1133_waitUntilSleep();
        if (ret != SI1133_OK) {
            return ret;
        }
        /* Skip if the command is RESET COMMAND COUNTER */
        if (command == SI1133_CMD_RESET_CMD_CTR) {
            break;
        }

        ret = SI1133_registerRead(SI1133_REG_RESPONSE0, &response);

        if ((response & SI1133_RSP0_COUNTER_MASK) == response_stored) {
            break;
        }
        else {
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
    ret = SI1133_registerWrite(SI1133_REG_COMMAND, command);
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

        ret = SI1133_registerRead(SI1133_REG_RESPONSE0, &response);
        if ((response & SI1133_RSP0_COUNTER_MASK) != response_stored) {
            break;
        }
        else {
            if (ret != SI1133_OK) {
                return ret;
            }
        }

        count++;
    }

    return SI1133_OK;
}

uint32_t SI1133_measurementForce(void)
{
    return SI1133_sendCmd(SI1133_CMD_FORCE_CH);
}

uint32_t SI1133_resetCmdCtr(void)
{
    return SI1133_sendCmd(SI1133_CMD_RESET_CMD_CTR);
}

uint32_t SI1133_measurementStart(void)
{
    return SI1133_sendCmd(SI1133_CMD_START);
}

uint32_t SI1133_paramRead(uint8_t address)
{

    uint8_t retval;
    uint8_t cmd;

    cmd = 0x40 + (address & 0x3F);

    retval = SI1133_sendCmd(cmd);
    if (retval != SI1133_OK) {
        return retval;
    }

    SI1133_registerRead(SI1133_REG_RESPONSE1, &retval);

    return retval;
}

uint32_t SI1133_getMeasurementf(float *lux, float *uvi)
{

    SI1133_Samples_TypeDef samples;
    uint32_t retval;

    /* Get the results */
    retval = SI1133_measurementGet(&samples);

    /* Convert the readings to lux */
    *lux = (float)SI1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    *uvi = (float)SI1133_getUv(samples.ch0, uk);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return retval;
}

uint32_t SI1133_getMeasurement(int32_t *lux, int32_t *uvi)
{

    SI1133_Samples_TypeDef samples;
    uint32_t retval;

    /* Get the results */
    retval = SI1133_measurementGet(&samples);

    /* Convert the readings to lux */
    int64_t v = SI1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = (int32_t)((v*1000) >> LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    v = SI1133_getUv(samples.ch0, uk);
    *uvi = (int32_t)((v*1000) >> UV_OUTPUT_FRACTION);
    
    return retval;
}

uint32_t SI1133_getIrqStatus(uint8_t *irqStatus)
{

    uint32_t retval;

    /* Read the IRQ status register */
    retval = SI1133_registerRead(SI1133_REG_IRQ_STATUS, irqStatus);

    return retval;
}

uint32_t SI1133_enableIrq0(bool enable)
{

    uint32_t retval;

    if (enable) {
        retval = SI1133_registerWrite(SI1133_REG_IRQ_ENABLE, 0x0F);
    }
    else {
        retval = SI1133_registerWrite(SI1133_REG_IRQ_ENABLE, 0);
    }

    //APP_ERROR_CHECK(retval);

    return retval;
}


/**
 * @brief get a measurement from SI1133
 * @param lux, ptr to where to put lux measurement
 * @param uvi, ptr to where to put uv measurement
 */
uint32_t SI1133_measureLuxUvif(float *lux, float *uvi)
{

    SI1133_Samples_TypeDef samples;
    uint32_t retval;
    uint8_t response;

    /* Force measurement */
    retval = SI1133_measurementForce();

    /* Go to sleep while the sensor does the conversion */
    os_time_delay(20);

    /* Check if the measurement finished, if not then wait */
    retval += SI1133_registerRead(SI1133_REG_IRQ_STATUS, &response);
    while (response != 0x0F) {
        os_time_delay(10);
        retval += SI1133_registerRead(SI1133_REG_IRQ_STATUS, &response);
    }

    /* Get the results */
    SI1133_measurementGet(&samples);
    //console_printf("Raw vals %d, %d, %d, %d\r\n", (int)samples.ch0, (int)samples.ch1, (int)samples.ch2, (int)samples.ch3);
    /* Convert the readings to lux */
    *lux = (float)SI1133_getLux(samples.ch1, samples.ch3, samples.ch2, &lk);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    *uvi = (float)SI1133_getUv(samples.ch0, uk);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return retval;
}

uint32_t SI1133_measurementPause(void)
{
    return SI1133_sendCmd(SI1133_CMD_PAUSE_CH);
}

uint32_t SI1133_init(void){

    int rc;

    /*Allow some time for the sensor to power up*/
    os_time_delay(10);
    
    rc = SI1133_reset();

    os_time_delay(10);

    rc += SI1133_paramSet(SI1133_PARAM_CH_LIST, 0x0f);
    rc += SI1133_paramSet(SI1133_PARAM_ADCCONFIG0, 0x78);
    rc += SI1133_paramSet(SI1133_PARAM_ADCSENS0, 0x11);
    rc += SI1133_paramSet(SI1133_PARAM_ADCPOST0, 0x40);
    rc += SI1133_paramSet(SI1133_PARAM_ADCCONFIG1, 0x4d);
    rc += SI1133_paramSet(SI1133_PARAM_ADCSENS1, 0x91);
    rc += SI1133_paramSet(SI1133_PARAM_ADCPOST1, 0x40);
    rc += SI1133_paramSet(SI1133_PARAM_ADCCONFIG2, 0x41);
    rc += SI1133_paramSet(SI1133_PARAM_ADCSENS2, 0x91);
    rc += SI1133_paramSet(SI1133_PARAM_ADCPOST2, 0x50);
    rc += SI1133_paramSet(SI1133_PARAM_ADCCONFIG3, 0x4d);
    rc += SI1133_paramSet(SI1133_PARAM_ADCSENS3, 0x17);
    rc += SI1133_paramSet(SI1133_PARAM_ADCPOST3, 0x40);

    rc += SI1133_registerWrite(SI1133_REG_IRQ_ENABLE, 0x0f);

    uint8_t irq;
    rc += SI1133_getIrqStatus(&irq);

    return rc;
} 

