
#include "bmm150.h"
#if 0
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL   (-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL    (-16384)
#define BMM150_OVERFLOW_OUTPUT               (-32768)
#define BMM150_NEGATIVE_SATURATION_Z         (-32767)
#define BMM150_POSITIVE_SATURATION_Z         (32767)
#endif

#ifndef BMM150_OVERFLOW_OUTPUT_FLOAT
#define BMM150_OVERFLOW_OUTPUT_FLOAT         0.0f
#endif


float bmm150_compensate_xf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_x)
{
    float retval = 0;
    float process_comp_x0;
    float process_comp_x1;
    float process_comp_x2;
    float process_comp_x3;
    float process_comp_x4;

    /* Overflow condition check */
    if (mag_data_x == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
        return BMM150_OVERFLOW_OUTPUT_FLOAT;

    if ((data_rhall == 0) || (trim->dig_xyz1 == 0))
        return BMM150_OVERFLOW_OUTPUT_FLOAT;
        
    process_comp_x0 = (((float)trim->dig_xyz1) * 16384.0f / data_rhall);
    retval = (process_comp_x0 - 16384.0f);
    process_comp_x1 = ((float)trim->dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_x2 = process_comp_x1 + retval * ((float)trim->dig_xy1) / 16384.0f;
    process_comp_x3 = ((float)trim->dig_x2) + 160.0f;
    process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
    retval = ((process_comp_x4 / 8192.0f) + (((float)trim->dig_x1) * 8.0f)) / 16.0f;

    return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer y axis data(micro-tesla) in float.
 */
float bmm150_compensate_yf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_y)
{
    float retval = 0;
    float process_comp_y0;
    float process_comp_y1;
    float process_comp_y2;
    float process_comp_y3;
    float process_comp_y4;
 
    if (mag_data_y == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) 
        return  BMM150_OVERFLOW_OUTPUT_FLOAT;

    if ((data_rhall == 0) || (trim->dig_xyz1 == 0))
        return  BMM150_OVERFLOW_OUTPUT_FLOAT;

    process_comp_y0 = ((float)trim->dig_xyz1) * 16384.0f / data_rhall;
    retval = process_comp_y0 - 16384.0f;
    process_comp_y1 = ((float)trim->dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_y2 = process_comp_y1 + retval * ((float)trim->dig_xy1) / 16384.0f;
    process_comp_y3 = ((float)trim->dig_y2) + 160.0f;
    process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
    retval = ((process_comp_y4 / 8192.0f) + (((float)trim->dig_y1) * 8.0f)) / 16.0f;

    return retval;
}

float bmm150_compensate_zf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_z)
{
    float retval = 0;
    float process_comp_z0;
    float process_comp_z1;
    float process_comp_z2;
    float process_comp_z3;
    float process_comp_z4;
    float process_comp_z5;

    if (mag_data_z == BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) 
        return BMM150_OVERFLOW_OUTPUT_FLOAT;

    if ((trim->dig_z2 == 0) || (trim->dig_z1 == 0))
        return BMM150_OVERFLOW_OUTPUT_FLOAT;

    if ((trim->dig_xyz1 == 0) || (data_rhall == 0))
        return BMM150_OVERFLOW_OUTPUT_FLOAT;
   
    process_comp_z0 = ((float)mag_data_z) - ((float)trim->dig_z4);
    process_comp_z1 = ((float)data_rhall) - ((float)trim->dig_xyz1);
    process_comp_z2 = (((float)trim->dig_z3) * process_comp_z1);
    process_comp_z3 = ((float)trim->dig_z1) * ((float)data_rhall) / 32768.0f;
    process_comp_z4 = ((float)trim->dig_z2) + process_comp_z3;
    process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
    retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;

    return retval;
}


/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer X axis data(micro-tesla) in int16_t.
 */
int16_t bmm150_compensate_x(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_x)
{
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;

    if (mag_data_x == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
        return  BMM150_OVERFLOW_OUTPUT;

    if (data_rhall != 0)
        process_comp_x0 = data_rhall;
    else if (trim->dig_xyz1 != 0)
        process_comp_x0 = trim->dig_xyz1;
    else
        process_comp_x0 = 0;

    if (process_comp_x0 == 0)
        return  BMM150_OVERFLOW_OUTPUT;

    process_comp_x1 = ((int32_t)trim->dig_xyz1) * 16384;
    process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
    retval = ((int16_t)process_comp_x2);
    process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
    process_comp_x4 = (((int32_t)trim->dig_xy2) * (process_comp_x3 / 128));
    process_comp_x5 = (int32_t)(((int16_t)trim->dig_xy1) * 128);
    process_comp_x6 = ((int32_t)retval) * process_comp_x5;
    process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
    process_comp_x8 = ((int32_t)(((int16_t)trim->dig_x2) + ((int16_t)0xA0)));
    process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
    process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
    retval = ((int16_t)(process_comp_x10 / 8192));
    retval = (retval + (((int16_t)trim->dig_x1) * 8)) / 16;

    return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer Y axis data(micro-tesla) in int16_t.
 */
int16_t bmm150_compensate_y(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_y)
{
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;

    /* Overflow condition check */
    if (mag_data_y == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
        return  BMM150_OVERFLOW_OUTPUT;

    if (data_rhall != 0)
        process_comp_y0 = data_rhall;
    else if (trim->dig_xyz1 != 0)
        process_comp_y0 = trim->dig_xyz1;
    else
        process_comp_y0 = 0;

    if (process_comp_y0 == 0)
        return  BMM150_OVERFLOW_OUTPUT;

    process_comp_y1 = (((int32_t)trim->dig_xyz1) * 16384) / process_comp_y0;
    process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
    retval = ((int16_t)process_comp_y2);
    process_comp_y3 = ((int32_t) retval) * ((int32_t)retval);
    process_comp_y4 = ((int32_t)trim->dig_xy2) * (process_comp_y3 / 128);
    process_comp_y5 = ((int32_t)(((int16_t)trim->dig_xy1) * 128));
    process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
    process_comp_y7 = ((int32_t)(((int16_t)trim->dig_y2) + ((int16_t)0xA0)));
    process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
    process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
    retval = (int16_t)(process_comp_y9 / 8192);
    retval = (retval + (((int16_t)trim->dig_y1) * 8)) / 16;

    return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer Z axis data(micro-tesla) in int16_t.
 */
int16_t bmm150_compensate_z(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_z)
{
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;

    if (mag_data_z == BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL)
        return  BMM150_OVERFLOW_OUTPUT;
    
    if ((trim->dig_z1 == 0) || (trim->dig_z2 == 0))
        return  BMM150_OVERFLOW_OUTPUT;

    if ((data_rhall == 0) || (trim->dig_xyz1 == 0))
        return  BMM150_OVERFLOW_OUTPUT;

    /*Processing compensation equations*/
    process_comp_z0 = ((int16_t)data_rhall) - ((int16_t) trim->dig_xyz1);
    process_comp_z1 = (((int32_t)trim->dig_z3) * ((int32_t)(process_comp_z0))) / 4;
    process_comp_z2 = (((int32_t)(mag_data_z - trim->dig_z4)) * 32768);
    process_comp_z3 = ((int32_t)trim->dig_z1) * (((int16_t)data_rhall) * 2);
    process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
    retval = ((process_comp_z2 - process_comp_z1) / (trim->dig_z2 + process_comp_z4));

    /* saturate result to +/- 2 micro-tesla */
    if (retval > BMM150_POSITIVE_SATURATION_Z)
    {
        retval = BMM150_POSITIVE_SATURATION_Z;
    }
    else if (retval < BMM150_NEGATIVE_SATURATION_Z)
    {
        retval = BMM150_NEGATIVE_SATURATION_Z;
    }

    /* Conversion of LSB to micro-tesla*/
    retval = retval / 16;

    return (int16_t)retval;
}


