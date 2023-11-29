// When using 72Mhz on the MCU main clock, the fastest ADC capture time is 0.39 uS.
// If we use 2 ADCs in interleave mode we may get double the captures, that is .19 uS.
// However, due tothe deviations between ADC1 and ADC2, the waveform becomes jagged.
// There is a possibility to overclock the MCU main clock up to 120MHz.

#define ADC_CR1_FASTINT 0x70000         // Fast interleave mode DUAL MODE bits 19-16
volatile static bool dma1_ch1_Active;   // End of DMA indication
float samplingTime = 0;

void dmaadc_setup() {   //Setup ADC peripherals for interleaved continuous mode.
  int pinMapADCin = PIN_MAP[ad_ch0].adc_channel;
  int pinMapADCin2 = PIN_MAP[ad_ch1].adc_channel;

  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT;    // | ADC_CR2_DMA; // Set continuous mode and DMA
  // set ADC2 in regular simultaneous mode
  ADC1->regs->CR1 |= 0x60000;     
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  ADC2->regs->CR2 |= ADC_CR2_CONT;    // ADC 2 continuos
  ADC2->regs->SQR3 = pinMapADCin2;
}

void takeSamples() {
  // This loop uses dual interleaved mode to get the best performance out of the ADCs
  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);
  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS,
                     cap_buf32, DMA_SIZE_32BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
  dma_set_num_transfers(DMA1, DMA_CH1, NSAMP / 2);
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

void debug_print() {
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setCursor(12, 35);  display.print("      ");
  display.setCursor(12, 35);  display.print(samplingTime);
}

void adc_set_speed(void) {
  switch (rate) {
    case 0:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_1_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_1_5);
      dmaadc_setup();
      break;
    case 1:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_7_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_7_5);
      dmaadc_setup();
      break;
    case 2:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_28_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_28_5);
      dmaadc_setup();
      break;
    case 3:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_55_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_55_5);
      dmaadc_setup();
      break;
    case 4: 
    case 5:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_7_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_7_5);
      break;
    default:
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
      adc_set_sample_rate(PIN_MAP[ad_ch0].adc_device,ADC_SMPR_55_5);
      adc_set_sample_rate(PIN_MAP[ad_ch1].adc_device,ADC_SMPR_55_5);
      break;
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
