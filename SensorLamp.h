#ifndef _SENSORLAMP_H_
#define _SENSORLAMP_H_

#include <Arduino.h>
#include <MedianFilter.h>

#define FILTERSIZE             15    // Sample size for median filter

#define MIN_BRIGHTNESS          1    // Min brightness (0 - 255)
#define MAX_BRIGHTNESS        255    // Max brightness (0 - 255)

#define DEBOUNCE_CYCLES        30    // Number of cycles to debounce on / off
#define START_TRACKING_CYCLES 100    // Delay if hand is detected before starting tracking
#define END_TRACKING_CYCLES   100    // Delay if no hand is detected while tracking before stopping tracking
#define HAND_MINIMUM_CHANGE    10    // Minimal brightness change allowed
#define SENSE_MAX_OFFSET       50    // Defines offset from SENSE_MAX to prevent signal noise from triggering lamp


class SensorLamp
{
  public:
    SensorLamp(uint8_t sensorPin = A0, uint8_t ledPin = 13);
    ~SensorLamp();

    void init();
    void update(const uint32_t &cycleTime = 0);

    uint8_t handDetected(const uint32_t &cycleTime = 0);

    void getStatus(uint16_t &analog_in, uint8_t &pwm_out);

  private:
    MedianFilter filter;

    const uint8_t sensor_pin;
    const uint8_t led_pin;

    // in[] holds the measured analogRead() values for defined distances
    // Note: The in array should have increasing values
    const int16_t in[14]  = {90, 97, 105, 113, 124, 134, 147, 164, 185, 218, 255, 317, 414, 525};
    //const int16_t in[14]  = {  68,   70,   74,   78,   83,   87,  91,  94,  97, 101, 105, 145, 330, 340};
    // out[] holds the corresponding distances in mm
    const int16_t out[14] = {1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200};

    bool lamp_lighted = false;
    bool hand_tracking = false;

    uint16_t sense_min = 200;                   // Minimum sensing height in mm, must not be smaller then min value in out[]
    uint16_t sense_max = 1500;                  // Maximum sensing height in mm, must not be greater then max value in out[]
    uint16_t track_distance = 300;              // Distance in mm to move hand to go from min to max brightness

    uint16_t tracking_lower_limit = sense_min;
    uint16_t tracking_upper_limit = sense_max;

    uint8_t pwm_output = 0;

    uint8_t stored_bright = MAX_BRIGHTNESS;
    uint8_t target_bright = 0;

    uint16_t measure();
    void adjustBrightness(const uint8_t &level, const uint8_t &fade = true);
};

#endif // _SENSORLAMP_H_
