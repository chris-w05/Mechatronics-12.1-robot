#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

// LineSensor class using Pololu QTRSensors (for QTR-MD-08RC)
class LineSensor
{
public:
    static const uint16_t numberPins = 8;

    // startPin: first Arduino pin (pins must be provided consecutively)
    // calMin/calMax: arrays of length numberPins containing previously-saved bounds
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

    // Call from setup(). emitterPin default: QTRNoEmitterPin (255) -> no explicit emitter control.
    // timeout: max RC discharge wait in µs. Lowering this reduces read time at the cost of
    // accuracy on very dark surfaces. 1000 µs is reliable for typical competition mats.
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
    float getPosition() const { return _current_value; }
    const uint16_t *getRaw() const { return sensorValues; }
    const uint16_t *getCalibrated() const { return _calibrated; }

private:
    QTRSensors qtr;
    uint8_t _pins[numberPins];
    

    uint16_t _calMin[numberPins];
    uint16_t _calMax[numberPins];

    uint16_t sensorValues[numberPins] = {0};
    uint16_t _calibrated[numberPins] = {0};

    float _current_value = 0.0f;
    const float _distance_between_sensors = 0.8f; // cm
};