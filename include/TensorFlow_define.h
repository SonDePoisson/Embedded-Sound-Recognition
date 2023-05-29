#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "all_targets_model_data.h"

#define LABELS_COUNT 35

const char *labels[LABELS_COUNT] =
{
    "right",
    "eight",
    "cat",
    "tree",
    "backward",
    "learn",
    "bed",
    "happy",
    "go",
    "dog",
    "no",
    "wow",
    "follow",
    "nine",
    "left",
    "stop",
    "three",
    "sheila",
    "one",
    "bird",
    "zero",
    "seven",
    "up",
    "visual",
    "marvin",
    "two",
    "house",
    "down",
    "six",
    "yes",
    "on",
    "five",
    "forward",
    "off",
    "four"
};


// Create a memory pool for the nodes in the network
constexpr int tensor_pool_size = 80 * 1024;
uint8_t tensor_pool[tensor_pool_size];

// Define the model to be used
const tflite::Model* all_target_model;

// Define the interpreter
tflite::MicroInterpreter* interpreter;

// Input/Output nodes for the network
TfLiteTensor* input;
TfLiteTensor* output;