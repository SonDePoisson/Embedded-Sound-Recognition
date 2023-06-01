#include <Arduino.h>
#include <math.h>
// WiFi
#include "WiFi.h"
// I2S
#include "I2S_define.h"
// TensorFlow
#include "TensorFlow_define.h"
// Test Sound
#include "bird_sound.h"
// Audio
#include "Audio.h"

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

// MFCC


//--------------------------------------- Functions

void fill_buffer(int32_t raw_samples[], const unsigned char bird_sound[], int length, int offset)
{
  for (int i = 0; i < length; i++)
  {
    raw_samples[i] = (int32_t)bird_sound[i + offset];
  }
}

//--------------------------------------- Main


void setup() 
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\nStarting setup...");

  // // start up the I2S peripheral
  // I2S_init();

  // // start up the TensorFlow model
  // TF_init();

  // Serial.println("Setup Complete");
	// Serial.println("\nSpeak into the microphone to get a prediction\n");


  std::string wavFile = "/Users/clementpoisson/Desktop/CNN_MQTT_UFPR/data_speech_commands_v0.02/bird/0a9f9af7_nohash_0.wav";
  
}

int offset = 0;

void loop() 
{
  delay(1000); 

  
  // // read from the I2S device
  // size_t bytes_read = 0;
  // i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  // int samples_read = bytes_read / sizeof(int32_t);



  // while(offset < bird_sound_len - SAMPLE_BUFFER_SIZE)
  // {  
  //   Serial.printf("New Fill\n\n");

  //   fill_buffer(raw_samples, bird_sound, SAMPLE_BUFFER_SIZE, offset);

  //   TF_fill_input(raw_samples);

  //   TF_run_inference();

  //   TF_print_results(0.6);

  //   offset += SAMPLE_BUFFER_SIZE;
  // }
  
  // Serial.printf("None of the labels found\n");
  // exit(0);
}
