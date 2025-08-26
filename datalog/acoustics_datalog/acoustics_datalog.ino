/* Example for analogRead
*  You can change the number of averages, bits of resolution and also the comparison value or range.
*/


#include <ADC.h>
#include <ADC_util.h>
#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;

const int num_pins = 4;
const int pins[num_pins] = {A8,A7,A0,A14};
volatile bool flipflop_adc0 = 0; //to cycle between reading diff pins
volatile bool flipflop_adc1 = 0;
volatile bool flipflop_ram = 0; //switch ram1 or ram2

const uint32_t num_readings_ram2 = 42000;
const uint32_t num_readings_ram1 = 34000;
volatile uint32_t readings_cnt = 0;

ADC *adc = new ADC(); // adc object;

DMAMEM struct Data{ //7 bytes per line
  uint8_t id;
  uint32_t clk;
  uint16_t val;    //WILL OVERFLOW WITHIN 7 SECS
};
DMAMEM volatile struct Data buf[num_readings_ram2]; //record in 280ms bursts, use 1ms to write to SD
volatile struct Data buf2[num_readings_ram1];

void setup() {

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWriteFast(LED_BUILTIN,HIGH);

  for(uint8_t i=0; i<num_pins; i++) {
    pinMode(pins[i],INPUT);
  }
    
  Serial.begin(115200);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      while (1) {
      // No SD card, so don't do anything more - stay stuck here
      }
  }
  SD.remove("datalog1.txt");
  File dataFile = SD.open("datalog1.txt", FILE_WRITE_BEGIN);
  Serial.println("card initialized.");

  Serial.println("ADC setup");
  
  adc->adc0->setAveraging(1); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  adc->adc1->setAveraging(1); // set number of averages
  adc->adc1->setResolution(12); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  // // adc->adc0->stopTimer();
  // adc->adc0->startSingleRead(pins[0 + flipflop_adc0*2]);
  // adc->adc0->enableInterrupts(adc0_isr);
  // adc->adc0->startTimer(100000);

  // adc->adc1->stopTimer();
  // adc->adc1->startSingleRead(pins[1 + flipflop_adc1*2]);
  // adc->adc1->enableInterrupts(adc1_isr);
  // adc->adc1->startTimer(100000);
  while ((uint32_t)adc->adc0->analogRead(pins[1]) > 2000) {}  //wait for pulse
  delay(300); //WAIT FOR NEXT PULSE
  digitalWriteFast(LED_BUILTIN, LOW);

  elapsedMillis read_time;
  while (read_time<1000) {  //read for fixed time only
    if (!flipflop_ram) {
      buf[readings_cnt].id = 0;
      buf[readings_cnt].val = (uint32_t)adc->adc0->analogRead(pins[0]);
      buf[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf[readings_cnt].id = 1;
      buf[readings_cnt].val = (uint32_t)adc->adc1->analogRead(pins[1]);
      buf[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf[readings_cnt].id = 2;
      buf[readings_cnt].val = (uint32_t)adc->adc0->analogRead(pins[2]);
      buf[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf[readings_cnt].id = 3;
      buf[readings_cnt].val = (uint32_t)adc->adc1->analogRead(pins[3]);
      buf[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      if(num_readings_ram2-readings_cnt <= 3) {  //RAM2 buffer full, use RAM1 buffer
        flipflop_ram = 1;
        readings_cnt = 0;
      }
    } else {
      buf2[readings_cnt].id = 0;
      buf2[readings_cnt].val = (uint32_t)adc->adc0->analogRead(pins[0]);
      buf2[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf2[readings_cnt].id = 1;
      buf2[readings_cnt].val = (uint32_t)adc->adc1->analogRead(pins[1]);
      buf2[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf2[readings_cnt].id = 2;
      buf2[readings_cnt].val = (uint32_t)adc->adc0->analogRead(pins[2]);
      buf2[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      buf2[readings_cnt].id = 3;
      buf2[readings_cnt].val = (uint32_t)adc->adc1->analogRead(pins[3]);
      buf2[readings_cnt].clk = ARM_DWT_CYCCNT; readings_cnt++;
      if(num_readings_ram1-readings_cnt <= 3) {  //RAM1 buffer full, pause everything and transfer both buffers to SD
        Serial.printf("chunk: %lu\n",millis());
        for (uint32_t i=0; i<num_readings_ram2-1; i++) {
          dataFile.printf("%hhu,%lu,%u\n",buf[i].id,buf[i].clk,buf[i].val);
        }
        for (uint32_t i=0; i<readings_cnt; i++) {
          dataFile.printf("%hhu,%lu,%u\n",buf2[i].id,buf2[i].clk,buf2[i].val);
        }
        flipflop_ram = 0;
        readings_cnt = 0;
      }
    }
    
  }
  if (!flipflop_ram) {  //remaining values in RAM2 buffer only
    for (uint32_t i=0; i<readings_cnt; i++) { //send remaining buffer
      dataFile.printf("%hhu,%lu,%u\n",buf[i].id,buf[i].clk,buf[i].val);
    }
  } else {
    for (uint32_t i=0; i<num_readings_ram2-1; i++) {
      dataFile.printf("%hhu,%lu,%u\n",buf[i].id,buf[i].clk,buf[i].val);
    }
    for (uint32_t i=0; i<readings_cnt; i++) {
      dataFile.printf("%hhu,%lu,%u\n",buf2[i].id,buf2[i].clk,buf2[i].val);
    }
  }
  
  
  Serial.println("finish");
}


void loop() {
    
}

void adc0_isr() {
  uint16_t adc_val = adc->adc0->readSingle();
  buf[readings_cnt].id = flipflop_adc0*2;
  buf[readings_cnt].clk = ARM_DWT_CYCCNT;
  buf[readings_cnt].val = adc_val;
  readings_cnt++;
  flipflop_adc0 = !flipflop_adc0;
  adc->adc0->startSingleRead(pins[0 + flipflop_adc0*2]);
}

void adc1_isr() {
  uint16_t adc_val = adc->adc1->readSingle();
  buf[readings_cnt].id = (1 + flipflop_adc1*2);
  buf[readings_cnt].clk = ARM_DWT_CYCCNT;
  buf[readings_cnt].val = adc_val;
  readings_cnt++;
  flipflop_adc1 = !flipflop_adc1;
  adc->adc1->startSingleRead(pins[1 + flipflop_adc1*2]);
}

uint64_t cycles64()
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