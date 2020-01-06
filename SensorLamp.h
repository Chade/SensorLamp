#ifndef _SENSORLAMP_H_
#define _SENSORLAMP_H_

#include <Arduino.h>
#include <MedianFilter.h>

#define FILTERSIZE              15    // Sample size for median filter

#define BURST_DELAY_MICROS    1000
#define BURST_READINGS           8

#define MIN_BRIGHTNESS           1    // Min brightness (0 - 255)
#define MAX_BRIGHTNESS         255    // Max brightness (0 - 255)

#define BLINK_SHORT_MILLIS      20
#define BLINK_LONG_MILLIS      200

#define DEBOUNCE_MILLIS        200
#define DEBOUNCE_JUMP_MILLIS    50
#define START_TRACKING_MILLIS  500    // Delay if hand is detected before starting tracking
#define END_TRACKING_MILLIS    300    // Delay if no hand is detected while tracking before stopping tracking

#define SENSE_MAX_OFFSET        50    // Defines offset from SENSE_MAX to prevent signal noise from triggering lamp
#define MAX_JUMP               100

#define NOT_DETECTED             0
#define DETECTED                 1
#define TRACKING                 2


class SensorLamp
{
  public:
    SensorLamp(uint8_t sensorPin = A0, uint8_t ledPin = 13);
    ~SensorLamp();

    void init(const uint16_t &senseMin = 200, const uint16_t &senseMax = 1500, const uint16_t &trackDistance = 300);
    void update(const uint32_t &cycleTime = 0, const uint32_t &readoutTime = (BURST_READINGS * BURST_DELAY_MICROS / 1000), const bool burst = true);

    uint8_t handDetected(const uint32_t &readoutTime = (BURST_READINGS * BURST_DELAY_MICROS / 1000), const bool burst = true);

    void getStatus(uint16_t &analog_in, uint8_t &pwm_out);
    uint16_t getDistance();

  private:
    MedianFilter filter;

    const uint8_t sensor_pin;
    const uint8_t led_pin;

    // in[] holds the measured analogRead() values for defined distances
    // Note: The in array should have increasing values
    const int16_t in[14]  = {90, 97, 105, 113, 124, 134, 147, 164, 185, 218, 255, 317, 414, 525};

    // out[] holds the corresponding distances in mm
    // Note: The out array should have decreasing values
    const int16_t out[14] = {1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200};

    bool lamp_lighted = false;
    bool hand_tracking = false;

    uint16_t sense_min;                   // Minimum sensing height in mm, must not be smaller then min value in out[]
    uint16_t sense_max;                   // Maximum sensing height in mm, must not be greater then max value in out[]
    uint16_t track_distance;              // Distance in mm to move hand to go from min to max brightness

    uint16_t tracking_lower_limit;
    uint16_t tracking_upper_limit;

    uint16_t distance = 0;
    uint16_t last_distance = 0;

    uint32_t last_readout = 0;

    uint8_t pwm_output = 0;

    uint8_t stored_bright = MAX_BRIGHTNESS;
    uint8_t target_bright = 0;

    uint16_t measure();
    uint16_t burstMeasure();
    void adjustBrightness(const uint8_t &level, const bool fade = false, const uint16_t &fadeTime = 0);
};

#endif // _SENSORLAMP_H_
