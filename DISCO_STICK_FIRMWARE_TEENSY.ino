#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>

//Undefine this to disable serial comm
#define DBG_P(x) Serial.print(x)

const uint32_t N_LEDS = 32;
const uint32_t PIXEL_LIMIT = N_LEDS/2;
const uint8_t LED_CLK_PIN = 13;
const uint8_t LED_DATA_PIN = 11;
CRGBArray<N_LEDS> color_buffer;

void initializeLEDs() {
  DBG_P("LED Init\n");

  LEDS.addLeds<APA102, LED_DATA_PIN, LED_CLK_PIN, BGR, DATA_RATE_MHZ(20)>(color_buffer, N_LEDS);
  LEDS.setBrightness(20);
  // Cylon bootup pattern
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(128,0,0);
    FastLED.show();
    delay(20);
  }
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,128,0);
    FastLED.show();
    delay(20);
  }
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,0,128);
    FastLED.show();
    delay(20);
  }
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,0,0);
    FastLED.show();
    delay(20);
  }
  FastLED.show(); 
  DBG_P("OK\n");
}

void handleLEDs() {
  FastLED.show();
}
//NOTE: Patch teensy audio library, remove reference to internal analog reference
// GUItool: begin automatically generated code
AudioInputAnalog         adc1;
AudioAnalyzeFFT256       fft;
AudioConnection          patchCord(adc1, fft);
// GUItool: end automatically generated code

#define FFT_POINTS 256
#define FFT_BUCKETS 10
float spectra[FFT_BUCKETS];

void initializeAudio() {
  DBG_P("Audio Init\n");
  AudioMemory(12);
  DBG_P("OK\n");
}

void handleAudio() {
  if (fft.available()) {
    for(uint8_t bucket = 0; bucket < FFT_BUCKETS; ++bucket) {
      spectra[bucket] = fft.read(bucket);
    }
  }
}

const uint32_t ACCEL_CLK_PIN = 19;
const uint32_t ACCEL_DATA_PIN = 18;
Adafruit_MMA8451 accelerometer = Adafruit_MMA8451();
CRGB color;

void initializeMotion() {
  DBG_P("Motion Init\n");
  Wire.begin();
  TWBR = 152;
  if(accelerometer.begin()){
    accelerometer.setRange(MMA8451_RANGE_4_G);
    DBG_P("OK\n");
  } else {
    DBG_P("Failed\n");
  }
}

void handleMotion() {
  accelerometer.read();
  color = CRGB(accelerometer.x, accelerometer.y, accelerometer.z);
}
uint32_t heartbeat_interval = 1000;
uint32_t heartbeat_counter = 0;
bool heartbeat_type = false;
void setup() {
#ifdef DBG_P
  Serial.begin(115200);
  while(!Serial);
#endif
  DBG_P("Boot\n");

  initializeLEDs();
  initializeMotion();
  initializeAudio();

  DBG_P("Entering Main Loop\n");
}

void loop() {
  handleAudio();
  handleMotion();
  handleLEDs();
  if (++heartbeat_counter > heartbeat_interval) {
    Serial.println(heartbeat_type ? "x" : "X");
    heartbeat_type = !heartbeat_type;
    heartbeat_counter = 0;
  }
}

