/*
   STM32F103C8T6 Period Counter Library Version 2.04
   Copyright (c) 2024, Siliconvalley4066
   Licenced under the GNU GPL Version 3.0
*/
#include "PeriodCount_STM.h"

uint8_t PeriodCountClass::_prescaler = 8;      // use 1/8 prescaler or not
uint16_t PeriodCountClass::_gatetime = 1000;
uint16_t PeriodCountClass::_psc = 45;
uint16_t PeriodCountClass::_arr = 65535;
uint32 PeriodCountClass::_measure_time, PeriodCountClass::_timeout = 5000;
volatile uint16_t PeriodCountClass::_preva = 0;
volatile uint16_t PeriodCountClass::_prevb = 0;
volatile uint32 PeriodCountClass::_count = 0;
volatile uint32 csec, osec;
volatile bool PeriodCountClass::_ready = false;
bool PeriodCountClass::adjusted = false;

void PeriodCountClass::begin(uint16_t msec) {
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_1);  // timer2 input is normally A0. remap to A15
  _gatetime = msec;
  set_range();  // identify frequency around
  osec = millis();
  PeriodCountClass::init();
  timer_attach_interrupt(TIMER1, TIMER_CC1_INTERRUPT, capture_count);
  timer_enable_irq(TIMER1, TIMER_CC1_INTERRUPT);
  adjusted = true;
}

void PeriodCountClass::init(void) {
  uint16_t etpsdiv = getetpsdiv();
  // Timer1 count of modulo 2^16 & capture by trigger from Timer2
  timer_init(TIMER1);
  TIMER1->regs.gen->SMCR = TIMER_SMCR_SMS_TRIGGER | TIMER_SMCR_TS_ITR1; // capture trigger from timer 2
  TIMER1->regs.gen->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TRC; // Set CH1 to input capture
  TIMER1->regs.gen->CCER = TIMER_CCER_CC1E;             // Set CH1 to capture at rising edge
  TIMER1->regs.gen->CR1 = TIMER_CR1_ARPE; // auto reload enable
  TIMER1->regs.gen->ARR = 65535;  //count up to maximum (default)
  TIMER1->regs.gen->EGR = TIMER_EGR_UG;   //reread registers.
  // Timer3 count of modulo (2^16 - 1) & capture by trigger from Timer2
  timer_init(TIMER3);
  TIMER3->regs.gen->SMCR = TIMER_SMCR_SMS_TRIGGER | TIMER_SMCR_TS_ITR1; // capture trigger from timer 2
  TIMER3->regs.gen->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TRC; // Set CH1 to input capture
  TIMER3->regs.gen->CCER = TIMER_CCER_CC1E;             // Set CH1 to capture at rising edge
  TIMER3->regs.gen->CR1 = TIMER_CR1_ARPE; // auto reload enable
  TIMER3->regs.gen->ARR = 65534;  //count up to maximum - 1
  TIMER3->regs.gen->EGR = TIMER_EGR_UG;   //reread registers.
  // Timer2 setting the capture trigger for timer1 and timer3
  timer_init(TIMER2);
  TIMER2->regs.gen->CR1 = TIMER_CR1_ARPE | TIMER_CR1_URS;  // without interruptions
  TIMER2->regs.gen->CR2 = TIMER_CR2_MMS_UPDATE | TIMER_CR2_CCUS; // signal to capture operation of other timers
  TIMER2->regs.gen->SMCR = TIMER_SMCR_ECE | etpsdiv | TIMER_SMCR_MSM; // external clock mode 2 + divider/8
  TIMER2->regs.gen->PSC = _psc;
  TIMER2->regs.gen->ARR = _arr;
  TIMER2->regs.gen->EGR = TIMER_EGR_TG | TIMER_EGR_CC1G;
  TIMER2->regs.gen->EGR |= TIMER_EGR_UG;   //reread registers.

  TIMER1->regs.gen->CR1 |= TIMER_CR1_CEN; //start timer1
  TIMER3->regs.gen->CR1 |= TIMER_CR1_CEN; //start timer3
  TIMER2->regs.gen->CR1 |= TIMER_CR1_CEN; //start timer2
  PeriodCountClass::_measure_time = millis();
}

uint8_t PeriodCountClass::available(void) {
  uint8_t count_ready;
  if (!_ready) count_ready = 0;
  else if (adjusted) {
    count_ready = 0; adjusted = _ready = false;
    PeriodCountClass::_measure_time = millis();
  } else {
    count_ready = 1;
  }
  if ((millis() - PeriodCountClass::_measure_time) > _timeout) count_ready = 2;
  return count_ready;
}

uint32_t PeriodCountClass::read(void) {
  uint32_t count;

  if ((millis() - PeriodCountClass::_measure_time) > _timeout) {
    count = 0;
  } else {
    count = _count;
  }
  _ready = false;
  PeriodCountClass::_measure_time = millis();
  return count;
}

void PeriodCountClass::end(void) {
  timer_detach_interrupt(TIMER1, TIMER_CC1_INTERRUPT);
  timer_init(TIMER1);
  timer_init(TIMER2);
  timer_init(TIMER3);
  AFIO_BASE->MAPR &= ~AFIO_MAPR_TIM2_REMAP;     // cancel tim 2 Partial remap
}

void PeriodCountClass::setpre(int pre) {
  if (_prescaler == pre) return;
  _prescaler = pre;
  uint16_t etpsdiv = getetpsdiv();
  TIMER2->regs.gen->SMCR = TIMER_SMCR_ECE | etpsdiv | TIMER_SMCR_MSM; // external clock mode 2 + divider/8
}

void PeriodCountClass::gatetime(uint16_t msec) {
  _gatetime = msec;
}

void PeriodCountClass::timeout(uint32 msec) {
  _timeout = msec;
}

uint16_t PeriodCountClass::getetpsdiv(void) {
  uint16_t etpsdiv;
  if (_prescaler == 8) {
    etpsdiv = TIMER_SMCR_ETPS_DIV8;
  } else if (_prescaler == 4) {
    etpsdiv = TIMER_SMCR_ETPS_DIV4;
  } else if (_prescaler == 2) {
    etpsdiv = TIMER_SMCR_ETPS_DIV2;
  } else {
    etpsdiv = TIMER_SMCR_ETPS_OFF;
  }
  return etpsdiv;
}

double PeriodCountClass::countToFrequency(uint32_t count) {
  if (count == 0) return 0.0;
  if (count > 360000000L) count = 360000000L; // upper limit FCPU * 5sec
  return (double) ((uint32)_prescaler * (uint32)(_psc + 1) * ((uint32)_arr + 1)) * (double) F_CPU / (double) count;
}

bool PeriodCountClass::adjust(double freq) {
  uint16_t psc, arr;
  uint32 ifreq = freq * (double) _gatetime / 1000.0;
  if (ifreq < 288000000L) {
    if (ifreq > 650000) {
      setpre(8);          // set ETR prescaler to 8
      ifreq >>= 3;
    } else if (ifreq < 640000) {
      setpre(1);          // set ETR prescaler to 1
    } else if (_prescaler == 8) {
      ifreq >>= 3;
    }
    psc = ifreq >> 16;
    if (psc < 1) psc = 1;
    arr = ifreq / (psc + 1);
    if (arr > 1) arr--;
    if (arr < 1) arr = 1;
  } else {
    return false; // do nothing
  }
  uint16_t etpsdiv = getetpsdiv();
  TIMER2->regs.gen->SMCR = TIMER_SMCR_ECE | etpsdiv | TIMER_SMCR_MSM; // external clock mode 2 + divider/8
  _psc = psc; _arr = arr;
  if (TIMER2->regs.gen->ARR != _arr || TIMER2->regs.gen->PSC != _psc) {
    TIMER2->regs.gen->PSC = _psc;
    TIMER2->regs.gen->CNT = 0;
    TIMER2->regs.gen->ARR = _arr;
    TIMER2->regs.gen->EGR |= TIMER_EGR_UG;   //reread registers.
    adjusted = true;
  }
  return adjusted;
}

uint16_t PeriodCountClass::get_psc() {
  return _psc;
}

uint16_t PeriodCountClass::get_arr() {
  return _arr;
}

uint8_t PeriodCountClass::get_prescaler() {
  return _prescaler;
}

uint32 PeriodCountClass::mod_calc(uint16_t a, uint16_t b) {
  uint32 f = b - a;
  if (a > b) --f;
  f = (f << 16) + a;
  return (f);
}

void PeriodCountClass::capture_count(void) {
  uint16_t a, b, aa, bb;
  TIMER1->regs.gen->SR &= ~TIMER_SR_CC1IF;  // clear capture flag
  TIMER3->regs.gen->SR &= ~TIMER_SR_CC1IF;  // clear capture flag
  aa = TIMER1->regs.gen->CCR1;
  bb = TIMER3->regs.gen->CCR1;
  a = aa - _preva;
  if (bb < _prevb) {
    b = bb + 65535 - _prevb;
  } else {
    b = bb - _prevb;
  }
  _count = PeriodCountClass::mod_calc(a, b);
  _preva = aa;
  _prevb = bb;
  _ready = true;
  uint32 nsec = millis();
  csec = nsec - osec;
  osec = nsec;
  if (csec < 10) {    // less than 10msec is too short
    setpre(8);        // set ETR prescaler to 8
    _psc = 6;         // 1.8MHz - 360MHz (2sec - 10msec)
    _arr = 65535;
    TIMER2->regs.gen->PSC = _psc;
    TIMER2->regs.gen->ARR = _arr;
    TIMER2->regs.gen->EGR |= TIMER_EGR_UG;   //reread registers.
  }
}

uint16_t PeriodCountClass::freqcount(uint16_t msec) {
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_1);  // timer2 input is normally A0. remap to A15
  uint16_t etpsdiv = getetpsdiv();
  // Timer2 setting
  timer_init(TIMER2);
  Timer2.pause();
  TIMER2->regs.gen->SMCR = TIMER_SMCR_ECE | TIMER_SMCR_SMS_RESET | etpsdiv; // external clock mode 2 + divider/8
  Timer2.setPrescaleFactor(1);
  Timer2.setOverflow(65535);
  Timer2.refresh();
  Timer2.resume();
  delay(msec);
  Timer2.pause();
  return (Timer2.getCount());
}

void PeriodCountClass::set_range(void) {
  unsigned long count;
  // identify 360MHz - 8kHz
  setpre(8);
  count = 8000 * freqcount(1);    // 1msec gate
  // identify 655.350kHz - 10Hz
  setpre(1);
  if (count < 500000) {   // under 500kHz
    count = 10 * freqcount(100);  // 100msec gate
  }
  adjust((double) count);
}

PeriodCountClass PeriodCount;
