#pragma once
#include <map>

class Motor_Temperature_Converter {  // could not be a class if memory problems
public:
  float ADCToTemperature(uint16_t ADC) {
    // Conversion eq: R1(resistance) * (1 - ADC/ADCMax) = R2(series_resistance)*ADC/ADCMax
    float rhs = kSeriesResistance * ADC / kADCMax;
    float lhs = 1 - (ADC / kADCMax);
    float resistance = rhs / lhs;
    return interpolate(resistance_to_temperature_lut_, resistance);
  }

private:
  // consts from datasheet
  std::map<float, float> resistance_to_temperature_lut_{
      {980, -55},  {1030, -50}, {1135, -40}, {1190, -35}, {1247, -30}, {1367, -20}, {1430, -15},
      {1495, -10}, {1561, -5},  {1630, 0},   {1700, 5},   {1772, 10},  {1846, 15},  {1922, 20},
      {2000, 25},  {2080, 30},  {2161, 35},  {2245, 40},  {2330, 45},  {2417, 50},  {2506, 55},
      {2597, 60},  {2690, 65},  {2785, 70},  {2881, 75},  {2980, 80},  {3080, 85},  {3182, 90},
      {3286, 95},  {3392, 100}, {3499, 105}, {3607, 110}, {3714, 115}, {3817, 120}, {3915, 125},
      {4008, 130}, {4092, 135}, {4166, 140}, {4230, 145}, {4280, 150}, {4316, 155}};

  const float kSeriesResistance_{4700.0f};
  const float kADCMax_{1 << 15};  // 32768
};