#include <Arduino.h>
#include <math.h>
#include "stop_model_data.h"

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");
  delay(1000);
  Serial.println("Setup Complete");
}

void loop() 
{
    // Print the output of the model.
    Serial.println("Looping...");
    delay(1000);
}
