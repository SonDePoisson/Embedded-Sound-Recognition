#include <Arduino.h>
#include "TensorFlow_define.h"

extern constexpr int tensor_pool_size = 80 * 1024;
extern uint8_t tensor_pool[tensor_pool_size];
extern const tflite::Model* all_target_model;
extern tflite::MicroInterpreter* interpreter;
extern TfLiteTensor* input;
extern TfLiteTensor* output;

void TF_init()
{
    // Load the sample sine model
	all_target_model = tflite::GetModel(___builded_files_all_targets_tflite);
	Serial.println("TensorFlow model loaded!");

    // Define ops resolver and error reporting
	static tflite::ops::micro::AllOpsResolver resolver;
	static tflite::ErrorReporter* error_reporter;
	static tflite::MicroErrorReporter micro_error;
	error_reporter = &micro_error;

	// Instantiate the interpreter 
	static tflite::MicroInterpreter static_interpreter(
		all_target_model, resolver, tensor_pool, tensor_pool_size, error_reporter
	);

	interpreter = &static_interpreter;

	// Allocate the the model's tensors in the memory pool that was created.
	if(interpreter->AllocateTensors() != kTfLiteOk) {
		Serial.println("There was an error allocating the memory...ooof");
		return;
	}
	Serial.println("Memory allocation successful!");

    // Define input and output nodes
	input = interpreter->input(0);
	output = interpreter->output(0);
}