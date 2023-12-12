#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/ringbuf.h>
#include <arduinoFFT.h>

#define SAMPLES 1024

// Speaker
#define I2S_SPEAKER_BCK GPIO_NUM_27
#define I2S_SPEAKER_WS GPIO_NUM_26
#define I2S_SPEAKER_DATA_OUT GPIO_NUM_25
#define LED_PIN GPIO_NUM_33
#define ELECTROMAGNET_PIN GPIO_NUM_32

#define NUM_LEDS 24
#define INIT_BRIGHTNESS 100

#define BUFFER_SIZE 4
#define BUFFER_SLOT 128
#define BUFFER_TRIGGER 1

CRGB leds[NUM_LEDS];
RingbufHandle_t sample_buffer;
float *output_buffer, *input_buffer;

BluetoothA2DPSink a2dp_sink;
bool colorChanged = false;
int cont = 0;


void readStream(const uint8_t *data, uint32_t length) {
  cont++;

  int16_t *values = (int16_t*) data;

  int buff_idx = 0;
  size_t buffer_bytes_to_send = SAMPLES * sizeof(float);
  size_t bytes_sent;
  for (int i = 0; i < SAMPLES * 2; i+=2) {
    output_buffer[buff_idx] = (values[i] + values[i+1]) /2.0f;
    buff_idx++;
  }

  UBaseType_t res = xRingbufferSend(sample_buffer, output_buffer, buffer_bytes_to_send, pdMS_TO_TICKS(1000));
}
const float smoothingFactor = 0.2;
const float colorSmoothingFactor = 0.95;

void colorCalculation(void *parameters) {
  size_t bytes_received;
  size_t buffer_bytes_to_receive = SAMPLES * sizeof(float);

  float smoothedVal = 0;
  const int numSamples = 10; // Number of samples to consider for the running average
  float maxFreqBuffer[numSamples] = {0}; // Buffer to store max frequency values
  int bufferIndex = 0; // Index for circular buffer
  CRGB lastColor = CRGB::Black;
  uint32_t lastBeatTime = 0;
  const int beatThreshold = 200; // Adjust this threshold based on your signal characteristics
  bool beatDetected = false;
  unsigned long beatStartTime = 0;
  const int maxPulseDuration = 1000; // Base time duration for pulse in milliseconds
  int pulseDuration = maxPulseDuration; // Time duration for pulse in milliseconds
  const int settleDuration = 100; // Time duration for settling after the pulse

  while (1) {
    size_t item_byte_size;   
    input_buffer = (float *) xRingbufferReceive(sample_buffer, &item_byte_size, portMAX_DELAY);

    size_t item_size = item_byte_size / sizeof(float);

    if (input_buffer != NULL) {
      vRingbufferReturnItem(sample_buffer, (void *) input_buffer);
    } else {
      continue;
    }

    float vReal[item_size];
    float vImag[item_size]; 

    for (int i = 0; i < item_size; i++) {
      vReal[i] = input_buffer[i];
      vImag[i] = 0.0; 
    }

    float freq, val;
    ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, item_size, 44100);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    FFT.majorPeak(freq, val);

    smoothedVal = (smoothingFactor * val) + ((1 - smoothingFactor) * smoothedVal);

    // Store max frequency value in the buffer
    maxFreqBuffer[bufferIndex] = freq;
    bufferIndex = (bufferIndex + 1) % numSamples;

    // Calculate the running average of the max frequency
    float maxFreq = 0;
    for (int i = 0; i < numSamples; ++i) {
      maxFreq += maxFreqBuffer[i];
    }
    maxFreq /= numSamples;
    CRGB color;
    uint8_t brightness = map(smoothedVal, 0, INIT_BRIGHTNESS, 50, 255);
    Serial.println(freq);

    if (maxFreq > 20 && maxFreq < 1000) {
      color = blend(CRGB::Blue, CRGB::Red, map(maxFreq, 20, 1000, 0, 255));
    } else {
      color = blend(CRGB::White, lastColor, 200);
    }
    lastColor = color;

    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();

    // Beat detection and pulsating behavior within the same loop iteration
    if (freq >= 20 && freq <= 150 && !beatDetected) {
      beatDetected = true;
      pulseDuration = map(freq, 20, 150, 100, maxPulseDuration); 
      beatStartTime = millis(); // Record beat detection time
    }

    if (beatDetected) {
      unsigned long elapsedTime = millis() - beatStartTime;

      if (elapsedTime < pulseDuration) {
        analogWrite(ELECTROMAGNET_PIN, 255);
      } else {
        analogWrite(ELECTROMAGNET_PIN, 0);
        
        if (elapsedTime >= (pulseDuration + settleDuration)) { // Adjust the threshold for turning off
          // Reset variables after the pulse sequence
          beatDetected = false;
          beatStartTime = 0;
        }
      }
    } else {
      // No beat detected, electromagnet off
      analogWrite(ELECTROMAGNET_PIN, 0);
    }
  }
}

void setup() {
  Serial.begin(115200);

  static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = (i2s_bits_per_sample_t) 16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0, 
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_pin_config_t i2s_pin_config = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = I2S_SPEAKER_BCK,
      .ws_io_num = I2S_SPEAKER_WS,
      .data_out_num = I2S_SPEAKER_DATA_OUT,
      .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(i2s_pin_config);
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_stream_reader(readStream, true);
  a2dp_sink.start("Ferrofluid Speaker");
  
  // LEDS
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();

  pinMode(ELECTROMAGNET_PIN, OUTPUT);

  int ring_buffer_size    = BUFFER_SIZE    * SAMPLES * sizeof(float);
  int stream_buffer_trigger = BUFFER_TRIGGER * SAMPLES * sizeof(float);

  sample_buffer = xRingbufferCreateNoSplit(SAMPLES * sizeof(float), BUFFER_SIZE);

  output_buffer = (float*) malloc(SAMPLES * sizeof(float));
  input_buffer  = (float*) malloc(SAMPLES * BUFFER_TRIGGER * sizeof(float));

  xTaskCreatePinnedToCore(
    colorCalculation,
    "colorCalculation",
    25000,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  vTaskDelete(NULL);
}
