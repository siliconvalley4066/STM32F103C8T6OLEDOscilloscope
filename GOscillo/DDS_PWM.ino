/*
 * DDS Sine Generator mit ATMEGS 168
 * Timer4 generates the  35.15625 KHz Clock Interrupt
 *
 * KHM 2009 /  Martin Nawrath
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 */

extern const unsigned char sine256[], saw256[], revsaw256[], triangle[], rect256[];
extern const unsigned char pulse20[], pulse10[], pulse05[], delta[], noise[];
extern const unsigned char gaussian_noise[], ecg[], sinc5[], sinc10[], sinc20[];
extern const unsigned char sine2harmonic[], sine3harmonic[], choppedsine[];
extern const unsigned char sinabs[], trapezoid[], step2[], step4[], chainsaw[];
unsigned char *wp;
const unsigned char * wavetable[] PROGMEM = {sine256, saw256, revsaw256, triangle, rect256,
  pulse20, pulse10, pulse05, delta, noise, gaussian_noise, ecg, sinc5, sinc10, sinc20,
  sine2harmonic, sine3harmonic, choppedsine, sinabs, trapezoid, step2, step4, chainsaw};
const char Wavename[][5] PROGMEM = {"Sine", "Saw", "RSaw", "Tri", "Rect",
  "PL20", "PL10", "PL05", "Dlta", "Nois", "GNoi", "ECG", "Snc1", "Snc2", "Snc3",
  "Sin2", "Sin3", "CSin", "Sabs", "Trpz", "Stp2", "Stp4", "Csaw"};
const byte wave_num = (sizeof(wavetable) / sizeof(&sine256));
long ifreq = 23841; // frequency * 100 for 0.01Hz resolution
byte wave_id = 0;
#define DDSPin PB9

// const double refclk=35156.25;  // 35.15625kHz at 72MHz
double refclk=35156.25;           // System clock is 72MHz

// variables used inside interrupt service declared as voilatile
volatile byte icnt;             // var inside interrupt
volatile unsigned long phaccu;  // pahse accumulator
volatile unsigned long tword_m; // dds tuning word m

void dds_setup() {
  pinMode(DDSPin, PWM);
  refclk = sys_clk / (double) (8*256);
  Setup_timer4();
  tword_m=pow(2,32)*ifreq*0.01/refclk; // calulate DDS new tuning word
  wp = (unsigned char *) wavetable[wave_id];
}

void dds_close() {
  pinMode(DDSPin, INPUT);
  Timer4.detachInterrupt(TIMER_CH4);
  Timer4.pause();
  Timer4.setMode(TIMER_CH4, TIMER_DISABLED);
}

void dds_set_freq() {
  double dfreq;
  dfreq = (double)ifreq*0.01;     // adjust output frequency
  tword_m=pow(2,32)*dfreq/refclk; // calulate DDS new tuning word
}

void rotate_wave(bool fwd) {
  if (fwd) {
    wave_id = (wave_id + 1) % wave_num;
  } else {
    if (wave_id > 0) --wave_id;
    else wave_id = wave_num - 1;
  }
  wp = (unsigned char *) wavetable[wave_id];
}

void set_wave(int id) {
  wave_id = id;
  wp = (unsigned char *) wavetable[wave_id];
}

//******************************************************************
// Timer4 setup
// set prscaler to 8, PWM mode, 72000000/8/256 = 35.15625kHz clock
void Setup_timer4() {
  pinMode(DDSPin, PWM);
  Timer4.pause();
  Timer4.setPrescaleFactor(8);          // 9MHz
  Timer4.setOverflow(255);              // max count 35.15625kHz sampling
  Timer4.setMode(TIMER_CH4,TIMER_PWM);
  Timer4.setCompare(TIMER_CH4, 127);    // duty50%
  Timer4.attachInterrupt(TIMER_UPDATE_INTERRUPT, pwmISR); // interrupt on counter overflow
  Timer4.refresh();
  Timer4.resume();
}

//******************************************************************
// Timer4 Interrupt Service at 35.15625kHz = 28.4uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : ? microseconds ( inclusive push and pop)
void pwmISR(void) {
  phaccu=phaccu+tword_m;  // soft DDS, phase accu with 32 bits
  icnt=phaccu >> 24;      // use upper 8 bits for phase accu as frequency information
                          // read value fron ROM sine table and send to PWM DAC
  Timer4.setCompare(TIMER_CH4, wp[icnt]);
}

void update_ifrq(long diff) {
  long newFreq;
  int fast;
  if (diff != 0) {
    if (abs(diff) > 3) {
      fast = ifreq / 40;
    } else if (abs(diff) > 2) {
      fast = ifreq / 300;
    } else if (abs(diff) > 1) {
      fast = 25;
    } else {
      fast = 1;
    }
    if (fast < 1) fast = 1;
    newFreq = ifreq + fast * diff;
  } else {
    newFreq = ifreq;
  }
  newFreq = constrain(newFreq, 1, 999999);
  if (newFreq != ifreq) {
    ifreq = newFreq;
    dds_set_freq();
  }
}

float set_freq(float dfreq) {
  long newfreq = 100.0 * dfreq;
  ifreq = constrain(newfreq, 1, 999999);
  dds_set_freq();
  return (dds_freq());
}

float dds_freq(void) {
  return (float)ifreq * 0.01;
}

void disp_dds_freq(void) {
  display.print(dds_freq(), 2); display.print("Hz");
}

void disp_dds_wave(void) {
  display.print(Wavename[wave_id]); 
}
