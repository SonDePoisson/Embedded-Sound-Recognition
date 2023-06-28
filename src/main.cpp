#include <Arduino.h>
#include <math.h>
// TensorFlow
#include "TensorFlow_define.h"
// Wave
#include "Wave.h"
// MFCC
#include "../python_mfcc_gen_from_clement/mfcclib.h"
#include "../python_mfcc_gen_from_clement/mfcclib.cpp"

//--------------------------------------- Variables

// TensorFlow 
constexpr int tensor_pool_size = 80 * 1024;
uint8_t tensor_pool[tensor_pool_size];
const tflite::Model* all_target_model;
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

// MFCC
#define __numCepstra  15
#define __numFilters  16
#define __samplingRate 8000
#define __winLength  256
#define __frameShift  46
#define __lowFreq  50
#define __highFreq  samplingRate/2
#define __FFT_SIZE 1024
#define __mfcc_step 368

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
  
  MFCC_INIT(__FFT_SIZE, __samplingRate, __numCepstra, __winLength, __frameShift, __numFilters, __lowFreq, highFreq);

  Serial.printf("Setup done\n");
}

int offset = 0;

void loop() 
{


  // TF_fill_input(raw_samples);
  // TF_run_inference();
  // TF_print_results(0.6);
  
  // Serial.printf("one of the labels found\n");
}
