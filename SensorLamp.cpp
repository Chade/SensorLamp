#include "SensorLamp.h"
#include <Arduino.h>

SensorLamp::SensorLamp(uint8_t type, uint8_t sensorPin, uint8_t ledPin) :
        filter(FILTERSIZE), sensor_pin(sensorPin), led_pin(ledPin)
{
    if (type == Sharp_200_1500)
    {
        in_out_size = 14;
        in  = new (int16_t[14]) {  90,   97,  105,  113,  124,  134, 147, 164, 185, 218, 255, 317, 414, 525};
        out = new (int16_t[14]) {1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200};
    }
    else if (type == Sharp_40_300)
    {
        in_out_size = 12;
        in  = new (int16_t[12]) { 81,  90, 102, 115, 136, 157, 186, 231, 296, 408, 605, 650};
        out = new (int16_t[12]) {300, 275, 250, 225, 200, 175, 150, 125, 100,  75,  50,  40};
    }

    pinMode(sensor_pin, INPUT);
    pinMode(led_pin, OUTPUT);
    analogWrite(led_pin, 0);
}

SensorLamp::~SensorLamp()
{

}

void SensorLamp::init(const uint16_t &senseMin, const uint16_t &senseMax, const uint16_t &trackDistance)
{
    sense_min = senseMin;
    sense_max = senseMax;
    track_distance = trackDistance;

    // Fill median filter
    for (uint8_t i = 0; i < filter.size(); i++)
    {
        filter.add(analogRead(sensor_pin));
        delay(10);
    }

    // Get median
    int16_t adc_input = filter.get();

    // Convert analog value to distance in mm
    uint16_t distance = MedianFilter::multiMap(adc_input, in, out, in_out_size);

    // Adjust max sense value
    if (distance > sense_max)
    {
        ;
    }
    else if (distance > (sense_min + track_distance + SENSE_MAX_OFFSET))
    {
        sense_max = distance - SENSE_MAX_OFFSET;
    }
    else if (distance > (sense_min + SENSE_MAX_OFFSET))
    {
        sense_max = distance - SENSE_MAX_OFFSET;
        track_distance = sense_max - sense_min;

        // Operation might be impeded
        analogWrite(led_pin, 255);
        delay(250);
        analogWrite(led_pin, 0);
    }
    else
    {
        // Operation not possible
        while (true)
        {
            pwm_output = pwm_output ? 0 : 255;
            analogWrite(led_pin, pwm_output);
            delay(250);
        }
    }

    tracking_lower_limit = sense_min;
    tracking_upper_limit = sense_max;
}

void SensorLamp::update(const uint32_t &cycleTime, const uint32_t &readoutTime, const bool burst)
{
    uint32_t current_time = millis();
    static uint32_t debounce_time = 0;
    static uint32_t tracking_time = 0;
    static uint32_t blink_time = 0;

    // =========================================================================
    // Measure
    // =========================================================================
    if (current_time - last_readout >= readoutTime)
    {
        last_readout = current_time;
        last_distance = distance;

        if (burst)
          distance = burstMeasure();
        else
          distance = measure();
    }

    // =========================================================================
    // Debounce
    // =========================================================================
    int32_t debounce = (int32_t)(current_time - debounce_time);
    if (debounce < 0)
    {
        return;
    }
    else
    {
        debounce_time = 0;
    }

    // =========================================================================
    // Hand tracking active
    // =========================================================================
    if (hand_tracking)
    {
        int32_t delta = (int32_t)distance - (int32_t)last_distance;
        // *********************************************************************
        // Jump detected
        // *********************************************************************
        if ( delta > MAX_JUMP)
        {
            debounce_time = current_time + DEBOUNCE_JUMP_MILLIS;
        }
        // *********************************************************************
        // Hand detected
        // *********************************************************************
        else if (distance < sense_max)
        {
            if (distance < tracking_lower_limit)
            {
                target_bright = MIN_BRIGHTNESS;

                // Shift lower border
                tracking_lower_limit = distance;
                tracking_upper_limit = constrain(distance + track_distance, sense_min, sense_max);

                // Blink to signal
                blink_time = current_time + BLINK_SHORT_MILLIS;
            }
            else if (distance > tracking_upper_limit)
            {
                target_bright = MAX_BRIGHTNESS;

                // Shift upper border
                tracking_lower_limit = constrain(distance - track_distance, sense_min, sense_max);
                tracking_upper_limit = distance;

                // Blink to signal
                blink_time = current_time + BLINK_SHORT_MILLIS;
            }
            else
            {
                if (distance != last_distance)
                {
                    target_bright = map(distance, tracking_lower_limit, tracking_upper_limit, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            tracking_time = 0;
        }
        // *********************************************************************
        // No hand detected
        // *********************************************************************
        else
        {
            if (tracking_time != 0)
            {
                if (current_time - tracking_time >= END_TRACKING_MILLIS)
                {
                    // Stop hand tracking
                    hand_tracking = false;
                    tracking_time = 0;

                    // Save new brightness
                    stored_bright = target_bright;

                    // Blink to confirm
                    blink_time = current_time + BLINK_LONG_MILLIS;
                }
            }
            else
            {
                tracking_time = current_time;
            }
        }
        // *********************************************************************
    }
    // =========================================================================
    // Hand tracking inactive
    // =========================================================================
    else
    {
        // *********************************************************************
        // Hand detected
        // *********************************************************************
        if (distance < sense_max)
        {
            if (tracking_time != 0)
            {
                if (current_time - tracking_time >= START_TRACKING_MILLIS)
                {
                    // Set tracking range
                    tracking_lower_limit = constrain(distance - map(stored_bright, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 0, track_distance), sense_min, sense_max);
                    tracking_upper_limit = constrain(distance + map(stored_bright, MIN_BRIGHTNESS, MAX_BRIGHTNESS, track_distance, 0), sense_min, sense_max);

                    // Start hand tracking
                    hand_tracking = true;
                    tracking_time = 0;

                    // Blink to confirm
                    blink_time = current_time + BLINK_LONG_MILLIS;
                }
            }
            else
            {
              tracking_time = current_time;
            }
        }
        // *********************************************************************
        // No hand detected
        // *********************************************************************
        else
        {
            if (tracking_time != 0)
            {
                lamp_lighted = !lamp_lighted;
                target_bright = lamp_lighted ? stored_bright : 0;
                debounce_time = current_time + DEBOUNCE_MILLIS;
                tracking_time = 0;
            }
        }
        // *********************************************************************
    }

    // =========================================================================
    // Set brightness or blink if requested
    // =========================================================================
    if ((int32_t)(current_time - blink_time) < 0)
    {
        analogWrite(led_pin, 0);
    }
    else
    {
        blink_time = 0;
        adjustBrightness(target_bright, !hand_tracking, cycleTime);
    }
}

uint8_t SensorLamp::handDetected(const uint32_t &readoutTime, const bool burst)
{
    uint32_t current_time = millis();
    static uint32_t debounce_time = 0;
    static uint32_t tracking_time = 0;

    uint8_t detected = NOT_DETECTED;

    // =========================================================================
    // Measure
    // =========================================================================
    if (current_time - last_readout >= readoutTime)
    {
        last_readout = current_time;
        last_distance = distance;

        if (burst)
          distance = burstMeasure();
        else
          distance = measure();
    }

    // =========================================================================
    // Debounce
    // =========================================================================
    int32_t debounce = (int32_t)(current_time - debounce_time);
    if (debounce < 0)
    {
        return;
    }
    else
    {
        debounce_time = 0;
    }

    // =========================================================================
    // Hand detected
    // =========================================================================
    if (distance < sense_max)
    {
        if (tracking_time != 0)
        {
            if (current_time - tracking_time >= START_TRACKING_MILLIS)
            {
                detected = TRACKING;
            }
        }
        else
        {
            tracking_time = current_time;
        }
    }
    // =========================================================================
    // No hand detected
    // =========================================================================
    else
    {
        if (tracking_time != 0)
        {
            debounce_time = current_time + DEBOUNCE_MILLIS;
            tracking_time = 0;

            if (current_time - tracking_time >= START_TRACKING_MILLIS)
            {
                detected = TRACKING;
            }
            else
            {
                detected = DETECTED;
            }
        }
    }

    return detected;
}

uint16_t SensorLamp::measure()
{
    // Read analog value and add to median filter
    filter.add(analogRead(sensor_pin));
    uint16_t adc_input = filter.get();

    // Convert analog value to distance in mm
    return MedianFilter::multiMap(adc_input, in, out, in_out_size);
}

uint16_t SensorLamp::burstMeasure()
{
    // Take our burst readings
    uint16_t current_reading;
    uint16_t lowest_reading = 1024;

    for (uint16_t i = 0; i < BURST_READINGS; i++) {
        // Read analog value
        current_reading = analogRead(sensor_pin);

        if (current_reading < lowest_reading) {
            lowest_reading = current_reading;
        }

        // Delay a short time before taking another reading
        delayMicroseconds(BURST_DELAY_MICROS);
    }

    // Add reading to median filter
    filter.add(lowest_reading);
    uint16_t adc_input = filter.get();

    // Convert value to distance in mm
    return MedianFilter::multiMap(adc_input, in, out, in_out_size);
}

void SensorLamp::adjustBrightness(const uint8_t &level, const bool fade, const uint16_t &fadeTime)
{
    uint32_t current_time = millis();
    static uint32_t last_time = current_time;

    // Adjust brightness
    if (fade && pwm_output != level)
    {
        if (current_time - last_time >= fadeTime)
        {
            last_time = current_time;

            if (pwm_output > level && pwm_output > 0)
                --pwm_output;
            if (pwm_output < level && pwm_output < 0xFF)
                ++pwm_output;
        }
    }
    else
    {
        pwm_output = level;
    }
    analogWrite(led_pin, pwm_output);
}

void SensorLamp::getStatus(uint16_t &analog_in, uint8_t &pwm_out)
{
    analog_in = filter.get();
    pwm_out = pwm_output;
}

uint16_t SensorLamp::getDistance()
{
    return distance;
}
