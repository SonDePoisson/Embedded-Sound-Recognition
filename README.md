# CNN_MQTT_UFPR
This is an internship project for the UFPR (Curitiba, Brazil) to make a sound recognition by MQTT and CNN on an ESP32.

This project is devided in 2 parts :
- The first one in python to create a TensorFlow Lite model from the database (the datebase is not include in the git project, so you have to add it by yourself).
- The second one in C++ for the embedded model on the ESP32 and the I2S microphone reading (or the wavefile reading for the development of the project)


TODO :
-  Python part OK : have to check it in the ESP32

- Fix wavefile_read() in the main to get int16_t data and then treat it with the same MFCC than lolis
- Pass it through the TFLite model and get the result
