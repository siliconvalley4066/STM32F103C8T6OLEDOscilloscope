#ifndef PeriodCount_h
#define PeriodCount_h

#include <Arduino.h>

class PeriodCountClass {
  public:
    static void begin(uint16_t msec);
    static uint8_t available(void);
    static uint32_t read(void);
    static void end(void);
    static void setpre(int pre);
    static void gatetime(uint16_t msec);
    static double countToFrequency(uint32_t count);
    static void timeout(uint32_t msec);
    static bool adjust(double freq);
    static uint16_t get_psc(void);
    static uint16_t get_arr(void);
    static uint8_t get_prescaler();
    static uint16_t freqcount(uint16_t msec);

  private:
    static uint32_t mod_calc(uint16_t a, uint16_t b);
    static uint16_t getetpsdiv(void);
    static void init(void);
    static void capture_count(void);
    static void set_range(void);
    static uint16_t _gatetime;
    static uint8_t _prescaler;      // use 1/8 prescaler or not
    static uint16_t _psc;
    static uint16_t _arr;
    static uint32_t _measure_time;
    static uint32_t _timeout;
    static volatile uint16_t _preva;
    static volatile uint16_t _prevb;
    static volatile uint32_t _count;
    static volatile bool _ready;
    static bool adjusted;
};

extern PeriodCountClass PeriodCount;

#endif
