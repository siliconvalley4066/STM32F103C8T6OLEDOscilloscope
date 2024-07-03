#define PWMPin PA3

byte duty = 128;  // duty ratio = duty/256
byte p_range = 0;
unsigned short count;
const long range_min[17] PROGMEM = {1, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767};
const long range_div[17] PROGMEM = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

double pulse_frq(void) {          // 0.01676Hz <= freq <= 36MHz
  long divide = range_div[p_range];
  return(sys_clk / ((double)((long)count + 1) * (double)divide));
}

void set_pulse_frq(float freq) {
  if (freq > (float)(sys_clk / 2)) freq = sys_clk / 2;
  p_range = constrain(9 - int(10.0 - log(sys_clk / 32768.0 / freq)/log(2)), 0, 16);
  long divide = range_div[p_range];
  setCounter(divide);
  count = (float)sys_clk/freq/(float)divide - 1;
  Timer2.setOverflow(count);
  setduty();
}

void pulse_init() {
  long divide;
  p_range = constrain(p_range, 0, 16);
  divide = range_div[p_range];

  pinMode(PWMPin, PWM);
  Timer2.setPrescaleFactor(divide);
  Timer2.setOverflow(count);
  Timer2.setMode(TIMER_CH4,TIMER_PWM);
  setduty();
  Timer2.refresh();
//  Timer2.resume();
}

void update_frq(int diff) {
  int fast;
  long divide, newCount;

  if (abs(diff) > 3) {
    fast = 512;
  } else if (abs(diff) > 2) {
    fast = 128;
  } else if (abs(diff) > 1) {
    fast = 25;
  } else {
    fast = 1;
  }
  newCount = (long)count + fast * diff;

  if (newCount < range_min[p_range]) {
    if (p_range < 1) {
      newCount = 1;
    } else {
      --p_range;
      newCount = 65535;
    }
  } else if (newCount > 65535) {
    if (p_range < 16) {
      ++p_range;
      newCount = range_min[p_range];
    } else {
      newCount = 65535;
    }
  }
  divide = range_div[p_range];
  setCounter(divide);
  count = newCount;
  Timer2.setOverflow(count);
  setduty();
}

void disp_pulse_frq(void) {
  float freq = pulse_frq();
  if (freq < 10.0) {
    display.print(freq, 5);
  } else if (freq < 100.0) {
    display.print(freq, 4);
  } else if (freq < 1000.0) {
    display.print(freq, 3);
  } else if (freq < 10000.0) {
    display.print(freq, 2);
  } else if (freq < 100000.0) {
    display.print(freq, 1);
  } else if (freq < 1000000.0) {
    display.print(freq * 1e-3, 2); display.print('k');
  } else if (freq < 10000000.0) {
    display.print(freq * 1e-6, 4); display.print('M');
  } else {
    display.print(freq * 1e-6, 3); display.print('M');
  }
  display.print("Hz");
}

void disp_pulse_dty(void) {
  static bool sp = true;
  float fduty = duty*100.0/256.0;
  display.print(fduty, 1); display.print('%');
  if (fduty < 9.95) {
    if (sp) {
      display.print(' ');
      sp = false;
    }
  } else {
    sp = true;
  }
}

void setCounter(int divide) {
  Timer2.setPrescaleFactor(divide);
}

void pulse_start(void) {
  pinMode(PWMPin, PWM);
  Timer2.setPrescaleFactor(range_div[p_range]);
  Timer2.setOverflow(count);
  Timer2.setMode(TIMER_CH4,TIMER_PWM);
  setduty();
  Timer2.refresh();
  Timer2.resume(); 
}

void pulse_close(void) {
  pinMode(PWMPin, INPUT);
  Timer2.pause(); 
}

void setduty(void) {
  if (count < 2)
    Timer2.setCompare(TIMER_CH4, 1);  // for max frequency 50% duty only
  else
    Timer2.setCompare(TIMER_CH4, map((unsigned short)duty, 0, 255, 0, count));
}
