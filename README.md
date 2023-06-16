# CNN_MQTT_UFPR
This is an internship project for the UFPR to make a bird sound recognazion with an ESP32

This project is devided in 2 parts :
- The first one in python to create a TensorFlow Lite model from the database (the datebase is not include in the git project, so you have to add it by yourself).
- The second one in C++ for the embedded model on the ESP32 and the I2S microphone reading (or the wavefile reading for the development of the project)


TODO :
- Run MFCC_from_C in MFCC.ipynb to check if it's working 
- Run Classification & TFLite_converter

- Fix wavefile_read() in the main to get int16_t data and then treat it with the same MFCC than lolis
- Pass it through the TFLite model and get the result