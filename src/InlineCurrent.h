#include <Arduino.h>

#define _ADC_VOLTAGE 3.3f            // ADC 电压
#define _ADC_RESOLUTION 4095.0f      // ADC 分辨率
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

class CurrSense
{
  public:
    CurrSense(int Mot_Num);
    inline float readADCVoltageInline(const int pin) {
      uint32_t raw_adc = analogRead(pin);
      return raw_adc * _ADC_CONV;
    }
    void configureADCInline(const int pinA, const int pinB, const int pinC);
    void calibrateOffsets();
    void init();
    void getPhaseCurrents();
    float current_a, current_b, current_c;
    int pinA;
    int pinB;
    int pinC;
    float offset_ia;
    float offset_ib;
    float offset_ic;
    const float _shunt_resistor;
    const float amp_gain;
    const float volts_to_amps_ratio;
    
    float gain_a;
    float gain_b;
    float gain_c;
    
    bool usePhaseC;
  private:
    int _Mot_Num;
};
