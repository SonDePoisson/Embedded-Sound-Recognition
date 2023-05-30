#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "all_targets_model_data.h"

#define LABELS_COUNT 35

void TF_init();

// Set the input node to the user input
void TF_fill_input(int32_t raw_samples[]);

// Run inference on the input data
void TF_run_inference();

// Print the results of the inference if the output is above the threshold
void TF_print_results(float threshold);