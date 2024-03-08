// When using 72Mhz on the MCU main clock, the fastest ADC capture time is 0.39 uS.
// If we use 2 ADCs in interleave mode we may get double the captures, that is .19 uS.
// However, due to the deviations between ADC1 and ADC2, the waveform becomes jagged.
// There is a possibility to overclock the MCU main clock up to 120MHz.

#include <STM32ADC.h>

#define ADC_CR1_FASTINT 0x70000         // Fast interleave mode DUAL MODE bits 19-16
volatile static bool dma1_ch1_Active;   // End of DMA indication
float samplingTime = 0;
STM32ADC myADC1(ADC1), myADC2(ADC2);

void dmaadc_setup() {   //Setup ADC peripherals for regular simultaneous mode.
  adc_set_reg_seqlen(ADC1, 1);
  adc_set_reg_seqlen(ADC2, 1);
  ADC1->regs->SQR3 = PIN_MAP[ad_ch0].adc_channel;
  ADC2->regs->SQR3 = PIN_MAP[ad_ch1].adc_channel;
  ADC1->regs->CR1 |= 0x60000;         // set ADC1 in regular simultaneous mode
  ADC1->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;  // ADC 1 continuos
  ADC2->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;  // ADC 2 continuos
}

void dmaadc_ilv_setup(byte ad_ch) {   //Setup ADC peripherals for Fast interleaved mode.
  adc_set_reg_seqlen(ADC1, 1);
  adc_set_reg_seqlen(ADC2, 1);
  ADC1->regs->SQR3 = PIN_MAP[ad_ch].adc_channel;
  ADC2->regs->SQR3 = PIN_MAP[ad_ch].adc_channel;
  ADC1->regs->CR1 |= ADC_CR1_FASTINT;   // set ADC1 in Fast Interleaved mode
  ADC1->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;  // ADC 1 continuos
  ADC2->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;  // ADC 2 continuos
}

void takeSamples() {
  // This loop uses regular simultaneous mode to get the best performance out of the ADCs
  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);
  adc_dma_enable(ADC1);
  myADC1.setDualDMA(cap_buf32, NSAMP / 2, DMA_MINC_MODE | DMA_TRNS_CMPLT);
  dma1_ch1_Active = 1;
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
//  samplingTime = micros();
  while (dma1_ch1_Active);
//  samplingTime = (micros() - samplingTime);
  dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  split_capture();
  int t = trigger_point();
  scaleDataArray(ad_ch0, t);
  scaleDataArray(ad_ch1, t);
//  debug_print();
}

void takeSamples_ilv(byte ad_ch) {
  // This loop uses regular simultaneous mode to get the best performance out of the ADCs
  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);
  adc_dma_enable(ADC1);
  myADC1.setDualDMA(cap_buf32, NSAMP / 4, DMA_MINC_MODE | DMA_TRNS_CMPLT);
  dma1_ch1_Active = 1;
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
//  samplingTime = micros();
  while (dma1_ch1_Active);
//  samplingTime = (micros() - samplingTime);
  dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  order_capture(ad_ch);
  int t = trigger_point();
  scaleDataArray(ad_ch, t);
//  debug_print();
}

void order_capture(byte ad_ch) {  // Swap word order to fix STM32 bug in packing 16bits into 32bits
  if (ad_ch == ad_ch0) {
    int16_t tmp;
    for (int i=0; i < NSAMP/2; i++) {
      if (i&1) {
        cap_buf[i] = tmp;
      } else {
        tmp = cap_buf[i];
        cap_buf[i] = cap_buf[i+1];
      }
    }
  } else {
    for (int i=0; i < NSAMP/2; i++) {
      if (i&1) {
        cap_buf1[i] = cap_buf[i-1];
      } else {
        cap_buf1[i] = cap_buf[i+1];
      }
    }
  }
}

void debug_print() {
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setCursor(12, 35);  display.print("      ");
  display.setCursor(12, 35);  display.print(samplingTime);
}

void adc_set_speed(void) {
  switch (rate) {
    case 0:
    case 1:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
      myADC1.setSampleRate(ADC_SMPR_1_5);
      myADC2.setSampleRate(ADC_SMPR_1_5);
      break;
    case 2:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);
      myADC1.setSampleRate(ADC_SMPR_7_5);
      myADC2.setSampleRate(ADC_SMPR_7_5);
      break;
    case 3:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);
      myADC1.setSampleRate(ADC_SMPR_28_5);
      myADC2.setSampleRate(ADC_SMPR_28_5);
      break;
    case 4:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
      myADC1.setSampleRate(ADC_SMPR_55_5);
      myADC2.setSampleRate(ADC_SMPR_55_5);
      break;
    case 5: 
    case 6:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
      myADC1.setSampleRate(ADC_SMPR_7_5);
      myADC2.setSampleRate(ADC_SMPR_7_5);
      break;
    default:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
      myADC1.setSampleRate(ADC_SMPR_55_5);
      myADC2.setSampleRate(ADC_SMPR_55_5);
      break;
  }
  if (rate <= RATE_ILV) {
    interleave_setup();
  } else if (rate <= RATE_DMA) {
    dmaadc_setup();
  } else {
//    myADC1.resetContinuous();
//    myADC2.resetContinuous();
  }
}

void interleave_setup(void) {
  if (ch0_mode != MODE_OFF) {
    dmaadc_ilv_setup(ad_ch0);
    trig_ch = ad_ch0;
  } else if (ch1_mode != MODE_OFF) {
    dmaadc_ilv_setup(ad_ch1);
    trig_ch = ad_ch1;
  }
}

void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}

void adc_dma_disable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 0);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

void split_capture() {
  for (int i = 0; i < NSAMP/2; ++i){
    cap_buf1[i] = cap_buf[i+i+1];
    cap_buf[i] = cap_buf[i+i];
  }
}

int trigger_point() {
  int trigger_ad, i;
  uint16_t *cap;

  if (trig_ch == ad_ch1) {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
    cap = cap_buf1;
  } else {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
    cap = cap_buf;
  }
  for (i = 0; i < (NSAMP/2 - SAMPLES); ++i) {
    if (trig_edge == TRIG_E_UP) {
      if (cap[i] < trigger_ad && cap[i+1] > trigger_ad)
        break;
    } else {
      if (cap[i] > trigger_ad && cap[i+1] < trigger_ad)
        break;
    }
  }
  return i;
}
