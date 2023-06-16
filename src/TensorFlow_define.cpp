#include <Arduino.h>
#include "TensorFlow_define.h"
#include "I2S_define.h"

const char *labels[LABELS_COUNT] =
{
    "right", "eight","cat","tree","backward","learn","bed","happy","go","dog","no",
    "wow","follow","nine","left","stop","three","sheila","one","bird","zero","seven","up",
    "visual","marvin","two","house","down","six","yes","on","five","forward","off","four"
};
extern constexpr int tensor_pool_size = 80 * 1024;
extern uint8_t tensor_pool[tensor_pool_size];
extern const tflite::Model* all_target_model;
extern tflite::MicroInterpreter* interpreter;
extern TfLiteTensor* input;
extern TfLiteTensor* output;

void TF_init()
{
    // Load the sample sine model
	all_target_model = tflite::GetModel(all_targets_tflite);
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

void TF_fill_input(int32_t raw_samples[])
{
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
      Serial.printf("Input(%d): %d\n", i, raw_samples[i]);
      input->data.f[i] = raw_samples[i];
    }
	Serial.printf("\n\n");
}


void TF_run_inference()
{
    if(interpreter->Invoke() != kTfLiteOk) {
      Serial.println("There was an error invoking the interpreter!");
      return;
    }
}

void TF_print_results(float threshold)
{
    for (size_t i = 0; i < LABELS_COUNT; i++)
    {
      if (output->data.f[i] > threshold && output->data.f[i] < 0.99)
      {
        Serial.print("=> ");
        Serial.printf("%-9s: %.3f\n", labels[i], output->data.f[i]);
        exit(0);
      }
    }
}