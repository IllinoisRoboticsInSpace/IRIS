/**
 * @file SparkMax.hpp
 * @brief Header file for the SparkMax class for controlling REV Robotics SPARK MAX controllers
 * @author Grayson Arendt
 */

#ifndef SPARKMAX_HPP
#define SPARKMAX_HPP
#pragma once

#include "SparkBase.hpp"

/**
 * @class SparkMax
 * @brief A class for controlling REV Robotics SPARK MAX via CAN bus
 *
 * This class provides methods to configure, control, and monitor the SPARK MAX.
 * It supports various control modes, parameter settings, and status readings.
 */
class SparkMax : public SparkBase
{
public:
  explicit SparkMax(const std::string & interfaceName, uint8_t deviceId);
  ~SparkMax() override = default;
};

#endif // SPARKMAX_HPP
