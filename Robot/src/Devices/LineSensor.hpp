#pragma once

#include <Arduino.h>

class LineSensor
{
public:
    LineSensor(int startPin) : _startPin(startPin) {}

    void init()
    {
        for (int i = 0; i < numberPins; i++)
        {
            pinMode(_startPin + i, INPUT);
        }
    }

    // Read all sensor values into the array
    void update()
    {
        for (int i = 0; i < numberPins; i++)
        {
            _values[i] = digitalRead(_startPin + i);
        }
    }

    // Returns a signed value representing line position
    // Negative = line on left, Positive = line on right
    int readValue()
    {
        long weightedSum = 0;
        int activeCount = 0;

        for (int i = 0; i < numberPins; i++)
        {
            if (_values[i])
            {
                // Center sensors around zero
                int weight = i - (numberPins - 1) / 2;
                weightedSum += weight;
                activeCount++;
            }
        }

        // No line detected
        if (activeCount == 0)
        {
            return 0;
        }

        return weightedSum / activeCount;
    }

    // Optional: get raw sensor values
    const int *getValue() const
    {
        return _values;
    }

private:
    int _startPin;
    static const int numberPins = 8;
    int _values[numberPins] = {0};
};
