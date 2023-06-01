#include <Arduino.h>
#include <math.h>
// WiFi
#include "WiFi.h"
// I2S
#include "I2S_define.h"
// TensorFlow
#include "TensorFlow_define.h"
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

// Audio
struct signal *signal_in;

//--------------------------------------- Functions


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

  wavefile_read((char *)"data_speech_commands_v0.02/bird/0a396ff2_nohash_0.wav", signal_in);

  Serial.printf("Signal size: %d\n\n", signal_in->size);

  for (int i = 0; i < signal_in->size; i++)
  {
    Serial.printf("Data : %f\n", signal_in->data[i]);
  }

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
