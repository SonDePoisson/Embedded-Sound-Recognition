#include <Arduino.h>
#include <math.h>
// WiFi
#include "WiFi.h"
// I2S
#include "I2S_define.h"
// TensorFlow
#include "TensorFlow_define.h"
// Wave
#include "Wave.h"
// MFCC


//--------------------------------------- Variables

// I2S  
int32_t raw_samples[SAMPLE_BUFFER_SIZE];

// TensorFlow 
constexpr int tensor_pool_size = 80 * 1024;
uint8_t tensor_pool[tensor_pool_size];
const tflite::Model* all_target_model;
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

// Audio
struct signal signal_in;

//--------------------------------------- Functions

void exit_if(int cond, const char *prefix)
{
  if (!cond) return;
  Serial.println(prefix);
  exit(0);
}

//--------------------------------------- Main


void setup() 
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\nStarting setup...");
  exit_if(!SPIFFS.begin(), "An Error has occurred while mounting SPIFFS");

  // // start up the I2S peripheral
  // I2S_init();

  // // start up the TensorFlow model
  // TF_init();

  wavefile_read((char *)"/sound.wav", &signal_in);

  
	exit(0);
}

int offset = 0;

void loop() 
{
  delay(1000); 

  
  // // read from the I2S device
  // size_t bytes_read = 0;
  // i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  // int samples_read = bytes_read / sizeof(int32_t);

  TF_fill_input(raw_samples);
  TF_run_inference();
  TF_print_results(0.6);
  
  // Serial.printf("one of the labels found\n");
  // exit(0);
}
