#include <Arduino.h> 
#include "InlineCurrent.h"


//  - shunt_resistor  - 分流电阻值
//  - gain  - 电流检测运算放大器增益
//  - phA   - A 相 adc 引脚
//  - phB   - B 相 adc 引脚
//  - phC   - C 相 adc 引脚（可选）


#define _ADC_VOLTAGE 3.3f            //ADC 电压
#define _ADC_RESOLUTION 4095.0f      //ADC 分辨率

// ADC 计数到电压转换比率求解
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

#define NOT_SET -12345.0
#define _isset(a) ( (a) != (NOT_SET) )

CurrSense::CurrSense(int Mot_Num)
  : _Mot_Num(Mot_Num),
    _shunt_resistor(0.01f),
    amp_gain(50.0f),
    volts_to_amps_ratio( 1.0f / _shunt_resistor / amp_gain)  // 或者 1.0f / _shunt_resistor / amp_gain
{
  if(Mot_Num == 0)
  {
    pinA = 39;
    pinB = 36;
    pinC = NOT_SET;  // 如果不直接采集 C 相则设为 NOT_SET
    gain_a = volts_to_amps_ratio * -1;
    gain_b = volts_to_amps_ratio * -1;
    gain_c = volts_to_amps_ratio;
  }
  if(Mot_Num == 1)
  {
    pinA = 35;
    pinB = 34;
    pinC = NOT_SET;  // 同上
    gain_a = volts_to_amps_ratio * -1;
    gain_b = volts_to_amps_ratio * -1;
    gain_c = volts_to_amps_ratio;
  }
  // 缓存是否使用 C 相 ADC 采集
  usePhaseC = _isset(pinC);
}

void CurrSense::configureADCInline(const int pinA, const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if(_isset(pinC)) pinMode(pinC, INPUT);
}

// 查找 ADC 零偏移量的函数
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 100;
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        if(usePhaseC) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }
    offset_ia /= calibration_rounds;
    offset_ib /= calibration_rounds;
    if(usePhaseC) offset_ic /= calibration_rounds;
}

void CurrSense::init(){
    configureADCInline(pinA, pinB, pinC);
    calibrateOffsets();
}


// 读取全部三相电流

void CurrSense::getPhaseCurrents(){
    // 缓存 ADC 读数减少函数调用开销
    float adcA = readADCVoltageInline(pinA);
    float adcB = readADCVoltageInline(pinB);
    current_a = (adcA - offset_ia) * gain_a;
    current_b = (adcB - offset_ib) * gain_b;
    if(usePhaseC){
       float adcC = readADCVoltageInline(pinC);
       current_c = (adcC - offset_ic) * gain_c;
    } else {
       current_c = -(current_a + current_b);
    }
}
