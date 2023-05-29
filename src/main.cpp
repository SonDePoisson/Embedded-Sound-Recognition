#include <Arduino.h>
#include <math.h>
#include "WiFi.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "all_targets_model_data.h"

// Create a memory pool for the nodes in the network
constexpr int tensor_pool_size = 80 * 1024;
uint8_t tensor_pool[tensor_pool_size];

// Define the model to be used
const tflite::Model* sine_model;

// Define the interpreter
tflite::MicroInterpreter* interpreter;

// Input/Output nodes for the network
TfLiteTensor* input;
TfLiteTensor* output;

void setup() 
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\nStarting setup...");

  // Load the sample sine model
	Serial.println("Loading Tensorflow model....");
	sine_model = tflite::GetModel(___builded_files_all_targets_tflite);
	Serial.println("Model loaded!");

  // Define ops resolver and error reporting
	static tflite::ops::micro::AllOpsResolver resolver;

	static tflite::ErrorReporter* error_reporter;
	static tflite::MicroErrorReporter micro_error;
	error_reporter = &micro_error;

	// Instantiate the interpreter 
	static tflite::MicroInterpreter static_interpreter(
		sine_model, resolver, tensor_pool, tensor_pool_size, error_reporter
	);

	interpreter = &static_interpreter;

	// Allocate the the model's tensors in the memory pool that was created.
	Serial.println("Allocating tensors to memory pool");
	if(interpreter->AllocateTensors() != kTfLiteOk) {
		Serial.println("There was an error allocating the memory...ooof");
		return;
	}

	// Define input and output nodes
	input = interpreter->input(0);
	output = interpreter->output(0);
	Serial.println("Starting inferences... Input a number! ");

  Serial.println("Setup Complete");
}

void loop() 
{
  // Wait for serial input to be made available and parse it as a float
	if(Serial.available() > 0) {
    float user_input = Serial.parseFloat();
    	
    // Set the input node to the user input
    input->data.f[0] = user_input;

    // Run inference on the input data
    if(interpreter->Invoke() != kTfLiteOk) {
      Serial.println("There was an error invoking the interpreter!");
      return;
    }

    // Print the output of the model.
    Serial.print("Input: ");
    Serial.println(user_input);
    Serial.print("Output: ");
    Serial.println(output->data.f[0]);
    Serial.println("");
  }
}
