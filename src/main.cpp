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
#define __lowFreq      50
#define __highFreq     __samplingRate/2
#define __numFilters   16    
#define __numCepstra   15   
#define __samplingRate 8000
#define __FFT_SIZE     1024  // Sample
#define __frameShift   465   // Sample
int16_t vReal[__samplingRate] = {}; 
int16_t vReal_janela[__frameShift]; 
v_d mfcc_output, mfcc_output_frame;

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

  MFCC_INIT(__FFT_SIZE, __samplingRate, __numCepstra, __frameShift, __numFilters, __lowFreq, __highFreq);

  Serial.printf("Free Heap: %d\n", ESP.getFreeHeap());

  Serial.printf("Setup done\n");
}

void loop() 
{
  int input_buffer_index = 0;

  //receive waveform
  while(Serial.available())
  {
    // Serial.printf("Serial.available()\n");
    vReal[input_buffer_index]= (short) Serial.parseInt();
    input_buffer_index++;
  }  

  Serial.printf("End Reading\n");
  // send result
  if (input_buffer_index >= __samplingRate-1)
  {
    // // Serial.printf("input_buffer_index: %d >= 7999\n", input_buffer_index);
    for(int mfcc_index = 0; mfcc_index < 8000; mfcc_index+=__frameShift)
    {
      for(int array_copy_index = 0; array_copy_index< __frameShift; array_copy_index++)
      {
        vReal_janela[array_copy_index]=vReal[mfcc_index+array_copy_index];
        // Serial.printf("vReal_janela[%d] = %d\n", array_copy_index, vReal_janela[array_copy_index]);
      }

      mfcc_output_frame = mfcc_processFrame(vReal_janela, __frameShift, __frameShift, __FFT_SIZE, __numFilters, __numCepstra);
      mfcc_output.insert(mfcc_output.end(), mfcc_output_frame.begin(), mfcc_output_frame.end());

    }
    

    double arr[256];
    std::copy(mfcc_output.begin(), mfcc_output.end(), arr);


    for(int output_buffer_index=64; output_buffer_index < 320; output_buffer_index++)
      Serial.println(arr[output_buffer_index]);
  }
}