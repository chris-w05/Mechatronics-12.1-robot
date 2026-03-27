/**
 * @file LineSensor.hpp
 * @brief 8-channel reflective line sensor driver using the Pololu QTR-MD-08RC.
 *
 * Reads all eight RC-type sensors simultaneously and computes a weighted
 * average position of the detected line in centimetres, relative to the
 * sensor array's centre.  A negative position means the line is to the left;
 * a positive position means the line is to the right.
 */
#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

/**
 * @brief Pololu QTR-MD-08RC 8-channel line sensor driver.
 *
 * Computes a signed cm position offset of the line from the sensor array
 * centre.  Call `init()` once in setup and `update()` every iteration when
 * line-following mode is active (it blocks for up to `timeout` µs per call).
 */
class LineSensor
{
public:
    static const uint16_t numberPins = 8;

    /**
     * @brief Construct the sensor with pin and calibration arrays.
     * @param pins    Array of 8 Arduino pin numbers (left to right).
     * @param calMin  Per-sensor minimum raw values (obtained during calibration).
     * @param calMax  Per-sensor maximum raw values (obtained during calibration).
     */
    LineSensor(const uint16_t pins[numberPins], const uint16_t calMin[numberPins], const uint16_t calMax[numberPins])
    {
        for (uint8_t i = 0; i < numberPins; ++i)
        {
            // _calMin[i] = calMin[i];
            // _calMax[i] = calMax[i];
            _pins[i] = pins[i]; // OK because _pins is non-const; we're copying the values
            // Serial.println(_pins[i]);
        }
    }

    /**
     * @brief Initialise the QTR sensor library.
     * @param timeout    Maximum discharge time to wait per read cycle (µs).
     *                   Lower values reduce blocking time at the cost of accuracy on dark surfaces.
     * @param emitterPin Emitter control pin.  Use `QTRNoEmitterPin` (255) if not present.
     */
    void init(uint16_t timeout = 1000, uint8_t emitterPin = QTRNoEmitterPin)
    {
        qtr.setTypeRC();
        qtr.setTimeout(timeout);
        qtr.setSensorPins(_pins, numberPins);
    }

    /**
     * Read sensors and compute signed position in cm
     * */
    void update()
    {
        qtr.read(sensorValues);
        float sum = 0;
        float acc = 0;
        for (int i = 0; i < 8; i++)
        {
            // Serial.print("pin");
            // Serial.print(_pins[i]);
            // Serial.print(" ");
            // Serial.print(sensorValues[i]);
            // Serial.print("\t");
            sum += _distance_between_sensors * i * sensorValues[i];
            acc += sensorValues[i];
        }
        
        _current_value = sum/acc - 2.8;
        // Serial.println(_current_value);
    }
    
    /**
     * Gets the position of the line in centimeters
     * 
     * @returns the position of the line in cm
     */
    float getPosition() const { return -_current_value; }
    const uint16_t *getRaw() const { return sensorValues; }
    const uint16_t *getCalibrated() const { return _calibrated; }

private:
    QTRSensors qtr;              ///< Pololu QTR library sensor object
    uint8_t _pins[numberPins];   ///< Pin numbers for each sensor element

    uint16_t _calMin[numberPins]; ///< Calibration minimum per sensor
    uint16_t _calMax[numberPins]; ///< Calibration maximum per sensor

    uint16_t sensorValues[numberPins] = {0}; ///< Raw RC discharge counts from last update()
    uint16_t _calibrated[numberPins]  = {0}; ///< Normalised calibrated values (0–1000)

    float _current_value = 0.0f;          ///< Computed line position (cm, signed)
    const float _distance_between_sensors = 0.8f; ///< Centre-to-centre sensor spacing (cm)
};