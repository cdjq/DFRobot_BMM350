
/**
 * @file  DFRobot_BMM350.cpp
 * @brief  Define the infrastructure of the DFRobot_BMM350 class and the implementation of the underlying methods
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [GDuang](yonglei.ren@dfrobot.com)
 * @version     V1.0.0
 * @date        2024-05-06
 * @url         https://github.com/dfrobot/DFRobot_BMM350
 */

#include "DFRobot_BMM350.h"

static struct        bmm350_dev bmm350_sensor;
/*! Variable that holds the I2C device address selection */
static uint8_t dev_addr;
TwoWire *_pWire = NULL;
uint8_t bmm350_I2CAddr = 0;

void bmm350_delayUS(uint32_t period, void*intf_ptr)
{
    if(period > 1000){
        delay(period/1000);
    }else{
        delayMicroseconds(period);
    }
}

DFRobot_BMM350::DFRobot_BMM350(bmm350_read_fptr_t bmm350_readReg, bmm350_write_fptr_t bmm350_writeReg, bmm350_delay_us_fptr_t bmm350_delayUS, eBMM350_INTERFACE interface)
{
    switch(interface) {
        case eBMM350_INTERFACE_I2C: 
            dev_addr = BMM350_I2C_ADSEL_SET_LOW;
            bmm350_sensor.intf_ptr = &dev_addr;
            break;
        case eBMM350_INTERFACE_I3C: 
            break;
    }
    bmm350_sensor.read = bmm350_readReg;
    bmm350_sensor.write = bmm350_writeReg;
    bmm350_sensor.delay_us = bmm350_delayUS;
}

DFRobot_BMM350::~DFRobot_BMM350()
{
}

bool DFRobot_BMM350::sensorInit(void)
{
    return bmm350_init(&bmm350_sensor) == 0;
}

uint8_t DFRobot_BMM350::getChipID(void)
{
    return bmm350_sensor.chip_id;
}

void DFRobot_BMM350::softReset(void)
{
    bmm350_soft_reset(&bmm350_sensor);
    bmm350_set_powermode(BMM350_SUSPEND_MODE, &bmm350_sensor);

}

void DFRobot_BMM350::setOperationMode(enum bmm350_power_modes powermode)
{
    bmm350_set_powermode(powermode, &bmm350_sensor);
}

String DFRobot_BMM350::getOperationMode(void)
{
    String result;
    switch(bmm350_sensor.power_mode){
        case BMM350_SUSPEND_MODE:
            result = "bmm350 is suspend mode!";
            break;
        case BMM350_NORMAL_MODE:
            result = "bmm350 is normal mode!";
            break;
        case BMM350_FORCED_MODE:
            result = "bmm350 is forced mode!";
            break;
        case BMM350_FORCED_MODE_FAST:
            result = "bmm350 is forced_fast mode!";
            break;
        default:
            result = "error mode!";
            break;
    }
    return result;
}

void DFRobot_BMM350::setPresetMode(uint8_t presetMode, enum bmm350_data_rates rate)
{
    switch (presetMode){
        case BMM350_PRESETMODE_LOWPOWER:
            bmm350_set_odr_performance(rate, BMM350_NO_AVERAGING, &bmm350_sensor);
            break;
        case BMM350_PRESETMODE_REGULAR:
            bmm350_set_odr_performance(rate, BMM350_AVERAGING_2, &bmm350_sensor);
            break;
        case BMM350_PRESETMODE_ENHANCED:
            bmm350_set_odr_performance(rate, BMM350_AVERAGING_4, &bmm350_sensor);
            break;
        case BMM350_PRESETMODE_HIGHACCURACY:
            bmm350_set_odr_performance(rate, BMM350_AVERAGING_8, &bmm350_sensor);
            break;
        default:
            break;
    }
}

void DFRobot_BMM350::setRate(uint8_t rate)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t avg_odr_reg = 0;
    uint8_t avg_reg = 0;
    uint8_t reg_data = 0;

    bmm350_set_powermode(BMM350_NORMAL_MODE, &bmm350_sensor);

    switch(rate){
        case BMM350_DATA_RATE_1_5625HZ:
        case BMM350_DATA_RATE_3_125HZ:
        case BMM350_DATA_RATE_6_25HZ:
        case BMM350_DATA_RATE_12_5HZ:
        case BMM350_DATA_RATE_25HZ:
        case BMM350_DATA_RATE_50HZ:
        case BMM350_DATA_RATE_100HZ:
        case BMM350_DATA_RATE_200HZ:
        case BMM350_DATA_RATE_400HZ:
            /* Get the configurations for ODR and performance */
            rslt = bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &avg_odr_reg, 1, &bmm350_sensor);
            if (rslt == BMM350_OK){
                /* Read the performance status */
                avg_reg = BMM350_GET_BITS(avg_odr_reg, BMM350_AVG);
            }
            /* ODR is an enum taking the generated constants from the register map */
            reg_data = ((uint8_t)rate & BMM350_ODR_MSK);
            /* AVG / performance is an enum taking the generated constants from the register map */
            reg_data = BMM350_SET_BITS(reg_data, BMM350_AVG, (uint8_t)avg_reg);
            /* Set PMU command configurations for ODR and performance */
            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD_AGGR_SET, &reg_data, 1, &bmm350_sensor);
            if (rslt == BMM350_OK){
                /* Set PMU command configurations to update odr and average */
                reg_data = BMM350_PMU_CMD_UPD_OAE;
                /* Set PMU command configuration */
                rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &reg_data, 1, &bmm350_sensor);
                if (rslt == BMM350_OK){
                    rslt = bmm350_delay_us(BMM350_UPD_OAE_DELAY, &bmm350_sensor);
                }
            }
            break;
        default:
            break;
    }
}

float DFRobot_BMM350::getRate(void)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t avg_odr_reg = 0;
    uint8_t odr_reg = 0;
    float result = 0;

    /* Get the configurations for ODR and performance */
    rslt = bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &avg_odr_reg, 1, &bmm350_sensor);
    if (rslt == BMM350_OK){
        /* Read the performance status */
        odr_reg = BMM350_GET_BITS(avg_odr_reg, BMM350_ODR);
    }
    switch(odr_reg){
        case BMM350_DATA_RATE_1_5625HZ:
            result = 1.5625;
            break;
        case BMM350_DATA_RATE_3_125HZ:
            result = 3.125;
            break;
        case BMM350_DATA_RATE_6_25HZ:
            result = 6.25;
            break;
        case BMM350_DATA_RATE_12_5HZ:
            result = 12.5;
            break;
        case BMM350_DATA_RATE_25HZ:
            result = 25;
            break;
        case BMM350_DATA_RATE_50HZ:
            result = 50;
            break;
        case BMM350_DATA_RATE_100HZ:
            result = 100;
            break;
        case BMM350_DATA_RATE_200HZ:
            result = 200;
            break;
        case BMM350_DATA_RATE_400HZ:
            result = 400;
            break;
        default:
            break;
    }
  return result;
}

String DFRobot_BMM350::selfTest(eBMM350_SELFTEST testMode)
{
    int8_t rslt;
    String result;
    /* Structure instance of self-test data */
    struct bmm350_self_test st_data = { 0 };
    switch(testMode){
        case eBMM350_SELF_TEST_NORMAL:
            rslt = bmm350_perform_self_test(&st_data, &bmm350_sensor);
            if(rslt == 0){
                result = "xyz aixs self test success!";
            }else{
                result = "xyz aixs self test failed!";
            }
            bmm350_set_powermode(BMM350_SUSPEND_MODE, &bmm350_sensor); 
            break;
        case eBMM350_SELF_TEST_ADVANCED:
            result = "eBMM350_SELF_TEST_ADVANCED, To be realized!";
            break;
    }
    return result;
}

void DFRobot_BMM350::setMeasurementXYZ(enum bmm350_x_axis_en_dis en_x, enum bmm350_y_axis_en_dis en_y, enum bmm350_z_axis_en_dis en_z)
{
    bmm350_enable_axes(en_x, en_y, en_z, &bmm350_sensor);
}

String DFRobot_BMM350::getMeasurementStateXYZ(void)
{
    uint8_t axis_reg = 0;
    uint8_t en_x = 0;
    uint8_t en_y = 0;
    uint8_t en_z = 0;
    char result[100] = "";

    /* Get the configurations for ODR and performance */
    axis_reg = bmm350_sensor.axis_en;
    
    /* Read the performance status */
    en_x = BMM350_GET_BITS(axis_reg, BMM350_EN_X);
    en_y = BMM350_GET_BITS(axis_reg, BMM350_EN_Y);
    en_z = BMM350_GET_BITS(axis_reg, BMM350_EN_Z);
    
    strcat(result, (en_x == 1 ? "The x axis is enable! " : "The x axis is disable! "));
    strcat(result, (en_y == 1 ? "The y axis is enable! " : "The y axis is disable! "));
    strcat(result, (en_z == 1 ? "The z axis is enable! " : "The z axis is disable! "));
    return result;
}

sBmm350MagData_t DFRobot_BMM350::getGeomagneticData(void)
{   
    int8_t rslt;
    sBmm350MagData_t magData = {0};
    struct bmm350_mag_temp_data mag_temp_data = {0};
    bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &bmm350_sensor);
    magData.x = mag_temp_data.x;
    magData.y = mag_temp_data.y;
    magData.z = mag_temp_data.z;
    magData.temperature = mag_temp_data.temperature;
    magData.float_x = mag_temp_data.x;
    magData.float_y = mag_temp_data.y;
    magData.float_z = mag_temp_data.z;
    magData.float_temperature = mag_temp_data.temperature;
    return magData;
}

float DFRobot_BMM350::getCompassDegree(void)
{
  float compass = 0.0;
  sBmm350MagData_t magData = getGeomagneticData();
  compass = atan2(magData.x, magData.y);
  if (compass < 0) {
    compass += 2 * PI;
  }
  if (compass > 2 * PI) {
     compass -= 2 * PI;
  }
  return compass * 180 / M_PI;
}

void DFRobot_BMM350::setDataReadyPin(enum bmm350_interrupt_enable_disable modes, enum bmm350_intr_polarity polarity)
{
    /* Variable to get interrupt control configuration */
    uint8_t reg_data = 0;
    /* Variable to store the function result */
    int8_t rslt;
    /* Get interrupt control configuration */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &reg_data, 1, &bmm350_sensor);
    if (rslt == BMM350_OK)
    {
        reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_INT_MODE, BMM350_PULSED);
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_POL, polarity);
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OD, BMM350_INTR_PUSH_PULL);     
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OUTPUT_EN, BMM350_MAP_TO_PIN); 
        reg_data = BMM350_SET_BITS(reg_data, BMM350_DRDY_DATA_REG_EN, (uint8_t)modes);
        /* Finally transfer the interrupt configurations */
        rslt = bmm350_set_regs(BMM350_REG_INT_CTRL, &reg_data, 1, &bmm350_sensor);
    }
}

bool DFRobot_BMM350::getDataReadyState(void)
{
    int8_t rslt;
    uint8_t drdy_status;
    rslt = bmm350_get_interrupt_status(&drdy_status, &bmm350_sensor);
    if(drdy_status & 0x01){
        return true;
    }else{
        return false;
    }
}

void DFRobot_BMM350::setThresholdInterrupt(uint8_t modes, int8_t threshold, enum bmm350_intr_polarity polarity)
{
  if(modes == LOW_THRESHOLD_INTERRUPT){
    __thresholdMode = LOW_THRESHOLD_INTERRUPT;
    setDataReadyPin(BMM350_ENABLE_INTERRUPT, polarity);
    this->threshold = threshold;
  }else{
    __thresholdMode = HIGH_THRESHOLD_INTERRUPT;
    setDataReadyPin(BMM350_ENABLE_INTERRUPT, polarity);
    this->threshold = threshold;
  }
}

sBmm350ThresholdData_t DFRobot_BMM350::getThresholdData(void)
{
    sBmm350MagData_t magData = {0};
    thresholdData.mag_x = NO_DATA;
    thresholdData.mag_y = NO_DATA;
    thresholdData.mag_z = NO_DATA;
    thresholdData.interrupt_x = 0;
    thresholdData.interrupt_y = 0;
    thresholdData.interrupt_z = 0;
    bool state = getDataReadyState();
    if(state == true){
        magData = getGeomagneticData();
        if(__thresholdMode == LOW_THRESHOLD_INTERRUPT){
            if(magData.x < (int32_t)threshold*16){
                thresholdData.mag_x = magData.x;
                thresholdData.interrupt_x = 1;
            }
            if(magData.y < (int32_t)threshold*16){
                thresholdData.mag_y = magData.y;
                thresholdData.interrupt_y = 1;
            }
            if(magData.z < (int32_t)threshold*16){
                thresholdData.mag_z = magData.z;
                thresholdData.interrupt_z = 1;
            }
        }else if(__thresholdMode == HIGH_THRESHOLD_INTERRUPT){
            if(magData.x > (int32_t)threshold*16){
                thresholdData.mag_x = magData.x;
                thresholdData.interrupt_x = 1;
            }
            if(magData.y > (int32_t)threshold*16){
                thresholdData.mag_y = magData.y;
                thresholdData.interrupt_y = 1;
            }
            if(magData.z > (int32_t)threshold*16){
                thresholdData.mag_z = magData.z;
                thresholdData.interrupt_z = 1;
            }
        }
    }

    return thresholdData;
}

static int8_t bmm350_i2c_readData(uint8_t Reg, uint8_t *Data ,uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    _pWire->begin();
    int i=0;
    _pWire->beginTransmission(device_addr);
    _pWire->write(Reg);
    if(_pWire->endTransmission() != 0)
    {
        return -1;
    }
    _pWire->requestFrom(device_addr, (uint8_t)len);
    while (_pWire->available())
    {
        Data[i++]=_pWire->read();
    }
    return 0;
}

static int8_t bmm350_i2c_writeData(uint8_t Reg, const uint8_t *Data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    _pWire->begin();
    _pWire->beginTransmission(device_addr);
    _pWire->write(Reg);
    for(uint8_t i = 0; i < len; i++)
    {
        _pWire->write(Data[i]);
    }
    _pWire->endTransmission();
    return 0;
}

DFRobot_BMM350_I2C::DFRobot_BMM350_I2C(TwoWire *pWire, uint8_t addr): DFRobot_BMM350(bmm350_i2c_readData, bmm350_i2c_writeData, bmm350_delayUS, eBMM350_INTERFACE_I2C)
{
  _pWire = pWire;
  bmm350_I2CAddr = addr;
}

uint8_t DFRobot_BMM350_I2C::begin()
{
    _pWire->begin();
    _pWire->beginTransmission(bmm350_I2CAddr);
    if(_pWire->endTransmission() == 0){
        if(sensorInit()){
            return 0;
        }else{
            DBG("Chip id error ,please check sensor!");
            return 2;
        }
    }else{
        DBG("I2C device address error or no connection!");
        return 1;
    }
}




