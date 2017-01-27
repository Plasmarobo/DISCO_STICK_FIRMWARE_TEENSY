#include <Audio.h>
#include <Wire.h>
//#include <i2c_t3.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>
#include <math.h>

//Undefine this to disable serial comm
//#define DBG
#ifdef DBG
  #define DBG_P(x) Serial.print(x)
#else
  #define DBG_P(x)
#endif

//AUDIO CONSTANTS
//NOTE: Patch teensy audio library, remove reference to internal analog reference
// GUItool: begin automatically generated code
AudioInputAnalog         adc1;
AudioAnalyzeFFT256       fft;
AudioConnection          patchCord(adc1, fft);
// GUItool: end automatically generated code

#define FFT_POINTS 256
#define FFT_BUCKETS 10
float spectra_gain[FFT_BUCKETS] = {3.0f, 3.0f, 4.0f, 4.0f, 5.0f,
                        5.0f, 8.0f, 10.f, 15.f, 18.0f};
#define SPECTRA_HISTORY 10
float spectra_ring_buffer[SPECTRA_HISTORY][FFT_BUCKETS];
uint8_t spectra_index;
float audio_maxima;
const float audio_threshold = 0.05f;

//UTIL CONSTANTS
#define HEARTBEAT_PIN 6
uint8_t heartbeat_value;
const float heartbeat_min = 0.0f;
const float heartbeat_max = 140.0f;
uint8_t heartbeat_state = 0;
float heartbeat_counter;
float heartbeat_intervals[] = {1000.0f, 500.0f, 750.0f, 250.0f}; //ms

//LED CONSTANTS
const uint32_t N_LEDS = 144;
const uint32_t PIXEL_LIMIT = N_LEDS/2;
const uint8_t LED_CLK_PIN = 13;
const uint8_t LED_DATA_PIN = 11;
//CRGBArray<N_LEDS> color_buffer;
CRGB color_buffer[N_LEDS];

//ACCEL CONSTANTS
const uint32_t ACCEL_CLK_PIN = 19;
const uint32_t ACCEL_DATA_PIN = 18;
#define CPU_FREQ 72000000
#define TWI_FREQ 5000
const uint32_t I2C_FREQ = ((CPU_FREQ / TWI_FREQ) - 16) / 2;
Adafruit_MMA8451 accelerometer = Adafruit_MMA8451();
CRGB color;

void initializeHeartbeat() {
  pinMode(HEARTBEAT_PIN, OUTPUT);
  analogWrite(HEARTBEAT_PIN, heartbeat_min);
  heartbeat_value = heartbeat_min;
  heartbeat_counter = (float)millis();
}

void handleHeartbeat() {
  float amount = ((float)millis() - heartbeat_counter) / heartbeat_intervals[heartbeat_state];
  switch(heartbeat_state) {
    case 0: //Fade up from zero
      analogWrite(HEARTBEAT_PIN, (uint8_t)((heartbeat_max * amount) + heartbeat_min));
      break;
    case 1: //Hold for one interval
      analogWrite(HEARTBEAT_PIN, (uint8_t)(heartbeat_max + heartbeat_min));
      break;
    case 2: //Fade down
      analogWrite(HEARTBEAT_PIN, (uint8_t)((heartbeat_max - (heartbeat_max * amount)) + heartbeat_min));
      break;
    case 3: //Hold for one interval
      analogWrite(HEARTBEAT_PIN, (uint8_t)(heartbeat_min));
      break;
    default:
      analogWrite(HEARTBEAT_PIN, 0);
      break;
  }
  if (amount >= 1.0f) {
    heartbeat_counter = millis();
    if (++heartbeat_state > 3) {
      heartbeat_state = 0;
    }
  }
}

void initializeLEDs() {
  DBG_P("LED Init\n");

  LEDS.addLeds<APA102, LED_DATA_PIN, LED_CLK_PIN, BGR, DATA_RATE_MHZ(20)>(color_buffer, N_LEDS);
  LEDS.setBrightness(20);
  // bootup pattern
  for(uint32_t i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(128,0,0);
    FastLED.show();
    delay(5);
  }
  for(uint32_t i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,128,0);
    FastLED.show();
    delay(5);
  }
  for(uint32_t i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,0,128);
    FastLED.show();
    delay(5);
  }
  for(uint32_t i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,0,0);
    FastLED.show();
    delay(5);
  }
  FastLED.show(); 
  DBG_P("OK\n");
}

void handleLEDs() {
  float i, j;
  CRGB new_color;
  for(i = 0; i < FFT_BUCKETS; ++i) {
    for(j = 0; j < PIXEL_LIMIT; ++j) { 
      float mag =  (spectra_ring_buffer[spectra_index][(uint32_t)i] / audio_maxima) * abs(sin((2 * PI * (j /((float) PIXEL_LIMIT)) * (i + 1.0f)) + ((PI / 6) * ((uint32_t)i%4))));
      new_color = CRGB(color.r * mag, color.g * mag, color.b * mag);
      color_buffer[(uint32_t)j] = new_color;
      color_buffer[N_LEDS-1-(uint32_t)j] = new_color;
    }
  }
  FastLED.show();
}

void initializeAudio() {
  DBG_P("Audio Init\n");
  AudioMemory(12);
  DBG_P("OK\n");
}

void handleAudio() {
  uint8_t bucket;
  float local_maxima;
  if (fft.available()) {
    for(bucket = 0; bucket < FFT_BUCKETS; ++bucket) {
      spectra_ring_buffer[spectra_index][bucket] = spectra_gain[bucket] * fft.read(bucket);
    }
    ++spectra_index;
    if (spectra_index >= SPECTRA_HISTORY) {
      spectra_index = 0;
    }
    //Compute frame average maxima
    for(uint8_t frame = 0; frame < SPECTRA_HISTORY; ++frame) {
      local_maxima = audio_threshold;
      for(bucket = 0; bucket < FFT_BUCKETS; ++bucket) {
        local_maxima += spectra_ring_buffer[frame][bucket];
      }
      local_maxima /= FFT_BUCKETS;
      if (audio_maxima < local_maxima) {
        audio_maxima = local_maxima;
      }
    }
  }
}

void initializeMotion() {
  DBG_P("Motion Init\n");
  Wire.begin();
  //Wire.setClock(I2C_FREQ);
  TWBR = I2C_FREQ;
  if(accelerometer.begin()){
    accelerometer.setRange(MMA8451_RANGE_4_G);
    DBG_P("OK\n");
  } else {
    DBG_P("Failed\n");
  }
}

void handleMotion() {
  accelerometer.read();
  float x = (128.0f * accelerometer.x) / 8192.0f;
  float y = (128.0f * accelerometer.y) / 8192.0f;
  float z = (128.0f * accelerometer.z) / 8192.0f;
  color = CRGB(255 * x, 255 * y, 255 * z);
}

void setup() {
#ifdef DBG
  Serial.begin(115200);
  while(!Serial);
#endif
  DBG_P("Boot\n");

  initializeMotion();
  initializeAudio();
  initializeLEDs();

  DBG_P("Entering Main Loop\n");
}

void loop() {

  handleAudio();
  handleMotion();
  handleLEDs();
  handleHeartbeat();
}

