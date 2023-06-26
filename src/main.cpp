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
#include "../python_mfcc_gen_from_clement/mfcclib.h"

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

// MFCC

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

  unsigned int mfcc_step=368;
  int samplingRate = 8000;
  int FFT_SIZE=2048;

  int16_t vReal[samplingRate]; 
	double vReal_janela[FFT_SIZE];
  for (size_t i = 0; i < FFT_SIZE; i++)
  {
    Serial.printf("vReal[%d] : %hd\n", i, signal_in.data[i]);
  }
  
	v_d mfcc, mfcc_frame;
  // for(int mfcc_index = 0; mfcc_index < 8000; mfcc_index+=mfcc_step)
	// {
  //   for (int array_copy_index = 0; array_copy_index< mfcc_step; array_copy_index++)
  //   {
  //     vReal_janela[array_copy_index]=vReal[mfcc_index+array_copy_index];
	// 		// Serial.printf("vReal_janela[%d] : %hd\n", array_copy_index, vReal_janela[array_copy_index]);
	// 	}
	                      
	// 	// mfcc_frame =  mfcc_processFrame(vReal_janela, mfcc_step);
	// 	// mfcc.insert(mfcc.end(), mfcc_frame.begin(), mfcc_frame.end());

	// 	// Serial.printf("MFCC[%d] : %f\n", mfcc_index, mfcc[mfcc_index]);
	// }  
  
  
	exit_if(1, "\nSetup done");
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
}
