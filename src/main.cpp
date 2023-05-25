#include <Arduino.h>
#include <math.h>
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "stop_model_data.h"

const tflite::Model* stop_model;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Hello World!");

  // Load the sample sine model
	Serial.println("Loading Tensorflow model....");
	stop_model = tflite::GetModel(stop_model_data);
	Serial.println("Sine model loaded!");
}

void loop() 
{
}