#include <Arduino.h>
#include <math.h>
// WiFi
#include "WiFi.h"
// I2S
#include "I2S_define.h"
// TensorFlow
#include "TensorFlow_define.h"

// I2S Variables 
int32_t raw_samples[SAMPLE_BUFFER_SIZE];

// TensorFlow Variables
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
constexpr int tensor_pool_size = 80 * 1024;
uint8_t tensor_pool[tensor_pool_size];
const tflite::Model* all_target_model;
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;



void setup() 
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\nStarting setup...");

  // start up the I2S peripheral
  I2S_init();

  // start up the TensorFlow model
  TF_init();

  Serial.println("Setup Complete");

	Serial.println("\nSpeak into the microphone to get a prediction\n");
}

void loop() 
{
  // read from the I2S device
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  
  if (samples_read != SAMPLE_BUFFER_SIZE)
  {
    Serial.printf("Read %d samples from I2S device\n", samples_read);
  }
  else
  {
    // Set the input node to the user input
    for (int i = 0; i < samples_read; i++)
    {
      input->data.f[i] = raw_samples[i];
      // Serial.printf("Input(%d): %d\n", i, input->data.f[i]);
    }

    // Run inference on the input data
    if(interpreter->Invoke() != kTfLiteOk) {
      Serial.println("There was an error invoking the interpreter!");
      return;
    }

    // Print the output of the model.
    for (size_t i = 0; i < LABELS_COUNT; i++)
    {
      if (output->data.f[i] > 0.7)
        Serial.print("=> ");
      Serial.printf("%-9s: %9.3f\n", labels[i], output->data.f[i]);
    }

    Serial.printf("\n\n");

    delay(3000);
  }
}
