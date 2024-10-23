#pragma once
#include <map>

#include "utils/utils.hpp"

class MotorTemperatureConverter {  // could not be a class if memory problems
public:
  float adc_to_temperature(uint16_t adc) {
    float rhs = series_resistance_ * adc / adc_max_;
    float lhs = 1 - (adc / adc_max_);
    float resistance = rhs / lhs;
    return interpolate(resistance_to_temperature_lut_, resistance);
  }

private:
  std::map<float, float> resistance_to_temperature_lut_{
      {980, -55},  {1030, -50}, {1135, -40}, {1190, -35}, {1247, -30}, {1367, -20}, {1430, -15},
      {1495, -10}, {1561, -5},  {1630, 0},   {1700, 5},   {1772, 10},  {1846, 15},  {1922, 20},
      {2000, 25},  {2080, 30},  {2161, 35},  {2245, 40},  {2330, 45},  {2417, 50},  {2506, 55},
      {2597, 60},  {2690, 65},  {2785, 70},  {2881, 75},  {2980, 80},  {3080, 85},  {3182, 90},
      {3286, 95},  {3392, 100}, {3499, 105}, {3607, 110}, {3714, 115}, {3817, 120}, {3915, 125},
      {4008, 130}, {4092, 135}, {4166, 140}, {4230, 145}, {4280, 150}, {4316, 155}};

  const float series_resistance_{4700.0f};
  const float adc_max_{1 << 15};  // 32768
};

class InverterTemperatureConverter {
public:
  float adc_to_temperature(float adc) { return interpolate(adc_to_temperature_lut_, adc); }

private:
  std::map<float, float> adc_to_temperature_lut_{
      {28480, 125}, {28179, 120}, {27851, 115}, {27497, 110}, {27114, 105}, {26702, 100},
      {26261, 95},  {25792, 90},  {25296, 85},  {24775, 80},  {24232, 75},  {23671, 70},
      {23097, 65},  {22515, 60},  {21933, 55},  {21357, 50},  {20793, 45},  {20250, 40},
      {19733, 35},  {19247, 30},  {18797, 25},  {18387, 20},  {18017, 15},  {17688, 10},
      {17400, 5},   {17151, 0},   {16938, -5},  {16757, -10}, {16609, -15}, {16487, -20},
      {16387, -25}, {16308, -30}};
};