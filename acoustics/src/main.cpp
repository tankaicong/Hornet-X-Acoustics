#include <Arduino.h>
#include <ADC.h>
#include <ADC_util.h>
#include <IntervalTimer.h>
#include <complex>

const int BUFFER_SIZE = 128;  //DFT performed on first half of buffer (i.e. older data), when ping detected, regression performed on second half of buffer (i.e. newer data)
const int NUM_PINS = 3;
const int PINS[NUM_PINS] = {A8,A7,A0};
const int CENTRE_VALS[NUM_PINS] = {2059, 2058, 2053};
const int DFT_THRESHOLDS[NUM_PINS] = {1500, 1500, 1500};  //threshold at which DFT result considers a signal to be present

const double PINGER_FREQ = 45000;
const double PINGER_PERIOD = 1/PINGER_FREQ;
const double PINGER_MULTIPLIER = 0.000471238898038468985769396507492; //2*pi*PINGER_FREQ/TEENSY_FREQ
const double SAMPLING_FREQ = 125000;
const double SAMPLING_PERIOD = 1/SAMPLING_FREQ;
const double DFT_FREQ = 90000;  //do DFT check every half cycle of signal
const double DFT_PERIOD = 1/DFT_FREQ;
const int DFT_SIZE = BUFFER_SIZE/2;

const int REGRESSION_SIZE = BUFFER_SIZE/2;
const double HYDROPHONE_SPACING = 0.045; //distance between hydrophones in meters
const double SOUND_SPEED = 1500; //speed of sound in water in m/s
const double SOUND_WAVELENGTH = SOUND_SPEED/PINGER_FREQ;

typedef std::complex<double> cplx;

volatile int32_t bufs[NUM_PINS][BUFFER_SIZE]; //stores ADC readings
volatile uint64_t read_times[NUM_PINS][BUFFER_SIZE];  //stores clock cycle at time of sample (~should consider struct-ing this shit with vals)
uint8_t idx[NUM_PINS] = {0,0,0}; //points to TAIL of circular buffer, i.e. next value to overwrite
uint8_t idx_regression[NUM_PINS] = {0,0,0}; //points to 
cplx dft_results[NUM_PINS] = {0,0,0};
double phases[NUM_PINS] = {0,0,0};
double curr_angles[2];  //stores curr angles
double last_angles[2] = {0,-PI/6}; //stores last angle ; assumes starting position somewhere in front and downwards
uint8_t curr_pin = NUM_PINS-1;

IntervalTimer timer0; //1 timer to continuously request ADC readings from the 3 channels
ADC *adc = new ADC(); // adc object;
bool curr_adc = 1;


cplx sliding_dft(volatile int32_t *buf, uint32_t head, uint32_t detect_freq, double T);
double get_phase(volatile int32_t *buf, volatile uint64_t *times, uint32_t head);
void timer0_callback();
void adc0_isr();
void adc1_isr();
uint64_t cycles64();


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  for (uint8_t i=0; i<NUM_PINS; i++) {
    pinMode(PINS[i],INPUT);
  }
  Serial.begin(115200);

  Serial.println("Begin setup");
  adc->adc0->setAveraging(0); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  adc->adc1->setAveraging(0); // set number of averages
  adc->adc1->setResolution(12); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  Serial.println("End setup");
  timer0.begin(timer0_callback,SAMPLING_PERIOD/3.0*1000000);
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc1->enableInterrupts(adc1_isr);
}

elapsedMicros dft_time = 0;
elapsedMillis print_time = 0;
elapsedMillis comms_time = 0;
bool rising_edge = true;
bool pulse = false;

void loop() {
  if (pulse) {  //if 45khz pulse detected, do regression & direction finding

    //determine arrival order by manually comparing
    uint8_t arrival_case = -1;
    if (abs(dft_results[0])>abs(dft_results[1]) && abs(dft_results[0])>abs(dft_results[2])&&abs(dft_results[1]) > abs(dft_results[2])) arrival_case = 0; //1->2->3: 0 to -60
    else if (abs(dft_results[0])>abs(dft_results[1]) && abs(dft_results[0])>abs(dft_results[2])&&abs(dft_results[1]) < abs(dft_results[2])) arrival_case = 1; //1->3->2: 0 to 60
    else if (abs(dft_results[1])>abs(dft_results[0]) && abs(dft_results[1])>abs(dft_results[2])&&abs(dft_results[0]) > abs(dft_results[2])) arrival_case = 2; //2->1->3: -60 to -120
    else if (abs(dft_results[1])>abs(dft_results[0]) && abs(dft_results[1])>abs(dft_results[2])&&abs(dft_results[0]) < abs(dft_results[2])) arrival_case = 3; //2->3->1 -120 to -180
    else if (abs(dft_results[2])>abs(dft_results[0]) && abs(dft_results[2])>abs(dft_results[1])&&abs(dft_results[0]) > abs(dft_results[1])) arrival_case = 4; //3->1->2: 60 to 120
    else if (abs(dft_results[2])>abs(dft_results[0]) && abs(dft_results[2])>abs(dft_results[1])&&abs(dft_results[0]) < abs(dft_results[1])) arrival_case = 5; //3->2->1: 120 to 180

    for (uint8_t i=0; i<3; i++) { //do regression to get phases on all hydrophones
      phases[i] = get_phase(bufs[i], read_times[i], idx_regression[i]);
      Serial.printf("%f\n", phases[i]);
    }
    // phases[0]=1; phases[1]=2.25; phases[2]=1.5;  //for testing specific values
    double phi_3_2 = phases[2] - phases[1];
    double phi_3_1 = phases[2] - phases[0];
    double valid_angles[10][2];
    uint8_t valid_count = 0;
    bool found_valid = false;
    for (int8_t n=0; n<5; n++) {
      for (int8_t m=0; m<5; m++) {
        double a = (phi_3_1 + 2*PI*(m-2))/(phi_3_2 + 2*PI*(n-2));
        double azimuth = atan2(sqrt(3),1-2*a);
        if(azimuth > PI/2) azimuth -= PI;

        double tmp = (phi_3_2/(2*PI) + (n-2)) / sin(azimuth) * (SOUND_WAVELENGTH/HYDROPHONE_SPACING);
        if (tmp >= -1 && tmp <= 1) {
          double elevation = -acos(tmp);
          switch(arrival_case) {
            case 0: //1->2->3: 0 to -60
              if (azimuth <= 0 && azimuth >= -PI/3 && elevation >= -PI/2) found_valid = true;
              break;
            case 1: //1->3->2: 0 to 60
              if (azimuth >= 0 && azimuth <= PI/3 && elevation >= -PI/2) found_valid = true;
              break;
            case 2: //2->1->3: -60 to -120
              if ( (azimuth <= -PI/3 && azimuth >= -PI/4 && elevation >= -PI/2) || (azimuth >= PI/3 && azimuth <= PI/4 && elevation <= -PI/2) ) found_valid = true;
              break;
            case 3: //2->3->1 -120 to -180
              if (azimuth <= 0 && azimuth >= -PI/3 && elevation <= -PI/2) found_valid = true;
              break;
            case 4: //3->1->2: 60 to 120
              if ( (azimuth >= PI/3 && azimuth <= PI/4 && elevation >= -PI/2) || (azimuth <= -PI/3 && azimuth >= -PI/4 && elevation <= -PI/2) ) found_valid = true;
              break;
            case 5: //3->2->1: 120 to 180
              if (azimuth >= 0 && azimuth <= PI/3 && elevation <= -PI/2) found_valid = true;
              break;
            }
          if (found_valid) {
            valid_angles[valid_count][0] = azimuth;
            valid_angles[valid_count][1] = elevation;
            valid_count++;
            found_valid = false;
          }
          // Serial.printf("n: %d, m: %d, Az: %f, El: %f\n", n, m, azimuth*360/(2*PI), elevation*360/(2*PI));
        }
      }
    }
    
    double final_angle[2] = {-200,-200};
    double min_sq_dist = 1000000;
    if (valid_count==0) {
      Serial.println("No valid angles found");
    } else {
      for (int i=0; i<valid_count; i++) {
        double sq_dist = 2 - 2*(cos(valid_angles[i][1])*cos(last_angles[1])*cos(valid_angles[i][0]-last_angles[0]) + sin(valid_angles[i][1])*sin(last_angles[1]));
        if (sq_dist < min_sq_dist) {
          min_sq_dist = sq_dist;
          final_angle[0] = valid_angles[i][0];
          final_angle[1] = valid_angles[i][1];
        }
      }
      Serial.printf("Final Az: %f, Final El: %f\n", final_angle[0]*360/(2*PI), final_angle[1]*360/(2*PI));
    }
    pulse = false;
  } else { //if no pulse detected, continue with dft
    if (dft_time >= DFT_PERIOD*1000000) { 
      for(uint8_t i=0; i<NUM_PINS; i++) {
        dft_results[i] = sliding_dft(bufs[i], idx[i], PINGER_FREQ, SAMPLING_PERIOD);
      }
      dft_time = 0;
    }
  }
  

  if(rising_edge) {
    bool test = abs(dft_results[0]) >= DFT_THRESHOLDS[0] && abs(dft_results[1]) >= DFT_THRESHOLDS[1] && abs(dft_results[2]) >= DFT_THRESHOLDS[2];
    if (test && !pulse) { //if rising edge of 45khz detected AND pulse not detected
      Serial.printf("Rising at time: %lu\n", micros());
      for (uint8_t i=0; i<NUM_PINS; i++) {
        idx_regression[i] = (idx[i]+REGRESSION_SIZE)%BUFFER_SIZE;
      }
      pulse = true;
      rising_edge = false;
    }
  } else {
    bool test = abs(dft_results[0]) < DFT_THRESHOLDS[0] && abs(dft_results[1]) < DFT_THRESHOLDS[1] && abs(dft_results[2]) < DFT_THRESHOLDS[2];
    if (test) { //falling edge of 45khz signal when all 3 hydrophones freq fall below threshold
      Serial.printf("Falling at time: %lu\n", micros());
      pulse = false;
      rising_edge = true;
    }
  }

  // if(print_time>2000) {
  //   for(uint8_t i=0;i<3;i++) {
  //     Serial.printf("%d: %f, %f\n", i, abs(dft_results[i]), std::arg(dft_results[i]));
  //   }
  //   Serial.println();
  //   print_time = 0;
  // }
}

// performs dft at one specified frequency
cplx sliding_dft(volatile int32_t *buf, uint32_t head, uint32_t detect_freq, double T) {
  cplx total = 0;
  for (uint8_t i=0; i<DFT_SIZE; i++) {
    cplx v = cplx(buf[(i+head)%BUFFER_SIZE], 0) * std::exp(cplx(0,-2*PI*detect_freq*(double)i*T));
    total += v;
  }
  return total;
}

double get_phase(volatile int32_t *buf, volatile uint64_t *times, uint32_t head) {
  double C[REGRESSION_SIZE], S[REGRESSION_SIZE];
  int32_t y[REGRESSION_SIZE];
  for (uint8_t i = 0; i < REGRESSION_SIZE; i++) {
    y[i] = buf[(i+head)%BUFFER_SIZE];
    uint64_t t = times[(i+head)%BUFFER_SIZE];
    C[i] = cos(PINGER_MULTIPLIER*t);
    S[i] = sin(PINGER_MULTIPLIER*t); //do this way to avoid floating point errors from 2*pi*f directly
    // Serial.printf("%llu, %d\n", t,y[i]);
  }
  // Serial.printf("\n");
  double sum_C_sq = 0, sum_S_sq = 0, sum_CS = 0, sum_Cy = 0, sum_Sy = 0;
  for (uint8_t i = 0; i < REGRESSION_SIZE; i++) {
    double C_i = C[i], S_i = S[i];
    int32_t y_i = y[i];
    sum_C_sq += C_i*C_i;
    sum_S_sq += S_i*S_i;
    sum_CS += C_i*S_i;
    sum_Cy += C_i*y_i;
    sum_Sy += S_i*y_i;
  }
  double alpha = (sum_S_sq*sum_Cy - sum_CS*sum_Sy)/(sum_C_sq*sum_S_sq - sum_CS*sum_CS);
  double beta = (sum_C_sq*sum_Sy - sum_CS*sum_Cy)/(sum_S_sq*sum_C_sq - sum_CS*sum_CS);
  // Serial.printf("\n%f,%f,%f,%f,%f, %f, %f\n", sum_C_sq, sum_S_sq, sum_CS, sum_Cy, sum_Sy, alpha, beta);
  double phase = atan2(alpha,beta);
  return phase;
}

void timer0_callback() {
  curr_pin = (curr_pin+1)%NUM_PINS; //do pin change first before starting conversion
  curr_adc = !curr_adc;
  if(curr_adc==0) {
    adc->adc0->startSingleRead(PINS[curr_pin]);
  } else {
    adc->adc1->startSingleRead(PINS[curr_pin]);
  }
}

void adc0_isr() {
  uint8_t pin = ADC::sc1a2channelADC0[ADC1_HC0&0x1f];
  uint8_t i = 0;
  switch(pin) {
    case A8:
      break;
    case A7:
      i=1;
      break;
    case A0:
      i=2;
      break;
  }
  bufs[i][idx[i]] = adc->adc0->readSingle() - CENTRE_VALS[i];
  read_times[i][idx[i]] = cycles64();
  idx[i] = (idx[i]+1)%BUFFER_SIZE;
  
  if (adc->adc0->adcWasInUse) {
    // restore ADC config, and restart conversion
    adc->adc0->loadConfig(&adc->adc0->adc_config);
    // avoid a conversion started by this isr to repeat itself
    adc->adc0->adcWasInUse = false;
  }
  asm("DSB");
}

void adc1_isr() {
  uint8_t pin = ADC::sc1a2channelADC1[ADC2_HC0&0x1f];
  uint8_t i = 0;
  switch(pin) {
    case A8:
      //i=0;
      break;
    case A7:
      i=1;
      break;
    case A0:
      i=2;
      break;
  }
  bufs[i][idx[i]] = adc->adc1->readSingle() - CENTRE_VALS[i];
  read_times[i][idx[i]] = cycles64();
  idx[i] = (idx[i]+1)%BUFFER_SIZE;
  if (adc->adc1->adcWasInUse) {
    // restore ADC config, and restart conversion
    adc->adc1->loadConfig(&adc->adc1->adc_config);
    // avoid a conversion started by this isr to repeat itself
    adc->adc1->adcWasInUse = false;
  }
  asm("DSB");
}

uint64_t cycles64() //convert clock cycles register data from uint32 to uint64
{
    static uint32_t oldCycles = ARM_DWT_CYCCNT;
    static uint32_t highDWORD = 0;

    uint32_t newCycles = ARM_DWT_CYCCNT;
    if (newCycles < oldCycles)
    {
        ++highDWORD;
    }
    oldCycles = newCycles;
    return (((uint64_t)highDWORD << 32) | newCycles);
}