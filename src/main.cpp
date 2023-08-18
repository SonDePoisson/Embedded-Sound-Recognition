/*

  Testes Finais - Version 8.0
  
  Author: JoÃ£o Vitor Galdino Souto Alvares 
  Date: 13/07/2022
  
  Description: AtualizaÃ§Ã£o da versÃ£o finalGSD_v6.8 com a normalizaÃ§Ã£o da inferÃªncia.   
         
         -> ESP32
         
          -> With Debug
          
          This program is utilizing 214437 (16%) bytes of the memory FLASH
          The maximum is of 1310720 (1.3MB) bytes of memory FLASH.
          
          This program is utilizing 15404 (4%) bytes of the memory RAM
          The maximum is of 294912 (294KB) bytes of memory RAM.
          
          -> Without Debug
          
          This program is utilizing 213937 (16%) bytes of the memory FLASH
          The maximum is of 1310720 (1.3MB) bytes of memory FLASH.
          
          This program is utilizing 15404 (4%) bytes of the memory RAM
          The maximum is of 294912 (294KB) bytes of memory RAM.

*/

//*****************************************************************************************************************
// Library(ies)
//*****************************************************************************************************************

#include "EEPROM.h"
#include <SPI.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// To MFCC
#include <iostream>
#include <fstream>
#include <string.h>                                               // For memset
#include "../python_mfcc_gen_from_clement/mfcclib.h"
#include "../python_mfcc_gen_from_clement/mfcclib.cpp"

// To TFLite
// #include <TensorFlowLite_ESP32.h> // voir s il y a besoin
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"      // Incluir todas operaÃ§Ãµes
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/version.h"

// To import model
#include "../python_mfcc_gen_from_clement/all_targets_model_data.h" // pointer vers ton reseau

// To I2S
#include "driver/i2s.h"
                          
//*****************************************************************************************************************
// Contant(s)
//*****************************************************************************************************************

// Hardware mapping
#define pinToLED        2   
#define pinToRelay01      4                         
#define pinToRelay02      15  
#define pinCS1          33
#define pinCS2          27
#define pinCS3          10
#define pinWriteDAC01     25                          
#define pinWriteDAC02     26  
#define pinReadADC01      35
#define pinReadADC02      34
#define pinToInterrupt01    0
#define pinToInterrupt02    5
#define pinToCalibration    22
// To serial extern 
// To serial 1
#define pinToRX         14
#define pinToTX         12
#define RX1_extern        9
#define TX1_extern        10
#define RX2_extern        16
#define TX2_extern        17

// Macro(s) generic(s)
// To debug
#define DEBUG
#define DEBUG_CALIBRATION
#define DEBUG_INTERRUPT

// To time
#define timeToWait        1
#define keepAliveTimeToWait   1000
#define timeDefaultRelays   500
#define timeToJsonWait01    500
#define timeToJsonWait02    15000
#define timeToLEDFix      100
//#define timeToBlock     300000
#define timeToBlock       10000
#define maxKeepAlive      3
  
// To serial  
// #define SERIAL       38400
#define SERIAL          115200
#define CMDBUFFER_SIZE      64


// VARIÃVEIS DO PROTOCOLO:
/*
**************************************************
MODOS DE OPERAÃ‡ÃƒO:
0 - STANDALONE
1 - TELEMETRYHUB
DEFAULT: 1
**************************************************
INFERÃŠNCIA:
0 - 8
DEFAULT: 4
**************************************************
MODO DO RELÃ‰01:
SEM CONFIGURAÃ‡ÃƒO
**************************************************
TEMPO DO RELÃ‰01:
100ms - 5000ms
DEFAULT: 500ms
**************************************************
MODO DO RELÃ‰02:
0 - INATIVO
1 - ALARME
2 - WATCHDOG
DEFAULT: 2
**************************************************
TEMPO DO RELÃ‰02:
100ms - 5000ms
DEFAULT: 500ms
**************************************************
LED DE OPERAÃ‡ÃƒO:
0 - INATIVO
1 - ATIVO
DEFAULT: 1
**************************************************
GANHO:
0 - 100 (0.1 - 10)
DEFAULT: 10
*/

// Variables to GSD20 in the EEPROM
#define flagEEPROM        0
#define modeOperation     1                       // Default
#define inferenceValues     2                       // Default 95%
#define relay01Mode       3                       // Not change
#define timeRelay01Mode     4                       // Default 500ms
#define relay02Mode       6                       // Default Watchdog
#define timeRelay02Mode     7                       // Default 500ms
#define ledOperation      9                       // Default enable
#define gainAmpOp       10                        // Default 10
#define typeGSD         11
#define versionMainGSD      14
#define versionRevGSD     15
#define eventGSD        16
#define serialNumberFinalGSD20  17
#define modeCalibration     40
// To produciton
#define flagProduction      25
#define flagSetupMain     26

// Message of the Json
#define MAX_SIZE_JSON           250
#define MAX_SIZE_SN             7
#define MAX_EEPROM        50
#define headerGSD20       "HEADER"
#define status00GSD20     "status"
#define status01GSD20     "connect"
#define status02GSD20     "connectSucess"
#define status03GSD20     "connectFail"
#define status04GSD20     "keepalive"
#define status05GSD20     "config"
#define status06GSD20     "configSucess"
#define status07GSD20     "configFail"
#define status08GSD20     "factoryReset"
#define status09GSD20     "resetSucess"
#define typeNameGSD20     "TYPE"
#define typeGSD20       "GSD"
#define versionGSD20      "VERS"
#define firtVersionGSD20    "1.00"
#define serialNumberGSD20   "SN"
#define serialNumberTestGSD20 "SNTESTE"
#define serialNumberTestTGSD20  "1515150"
#define detectEvent       "EVNT"
#define passwordGSD20     "PW"
#define passwordTestGSD20   "s3nh4@mtwgsd"
#define modeOperationGSD20    "MODE"
#define inferenceGSD20      "INFE"
#define modeRelay01GDS20    "MRL1"
#define timeRelay01GSD20    "TRL1"
#define modeRelay02GDS20    "MRL2"
#define timeRelay02GSD20    "TRL2"
#define modeLEDGDS20      "MLED"
#define modeGainGDS20     "GAIN"

// To MCP41010
#define MAX_RESISTANCE      10000
#define WIPER_RESISTANCE    80
#define FIX_NORMALIZE     255
#define FIX_NORMALIZE_VARIABLE  2.4
#define MIN_VALUE_MCP41010    1

// To inteference
#define FIX_MAX_INFE      8
#define MAX_RATE        100
#define MIN_VALUE_INFE      0.1

// To RNN
// To buffer

// To assign variables // reconfigurer pour la 16X16
#define samplingRate 8000
#define lowFreq      50
#define highFreq     samplingRate/2
#define numCepstra   15   
#define numFilters   16    
#define FFT_SIZE     1024  // Sample
#define frameShift   465   // Sample
// #define winLength           128 // pas la paine pour la nouvelle mfcc lib de clement
// #define frameShift          92 // pas la paine pour la nouvelle mfcc lib de clement

// To serial
#define CMDBUFFER_SIZE      32
#define COMMAND_CONFIG      "A55AA5"

// To I2S
#define I2S_WS  25   // GPIO pin for Word Select signal
#define I2S_SD  33   // GPIO pin for Serial Data signal
#define I2S_SCK 32   // GPIO pin for Serial Clock signal
#define I2S_PORT I2S_NUM_0
#define bufferLen 8  // Number of buffers in the I2S buffer array

//*****************************************************************************************************************
// Prototype of the function(s)
//*****************************************************************************************************************

// To setup
void setupSerial();
void setupPins();
void setupEEPROM();
void setupMCP41010();
void setupMFCC();
void setupTFLite();
void setupTimer();
void disableTimer();
void setupInterrupt();
// void setupCalibration();
void setupI2S();

// To operate
void calculateInference();
void timerI2S();


// To verifier
void verifierJsonConnect();
void verifierJsonConfig();
void verifierKeepAlive();

// To send 
void sendJsonDefaut();
void sendJsonConnect();
void sendJsonConfig();
void sendJsonFailConnect();
void sendJsonFailConfig();
void sendKeepAlive();
void sendResetSucess();


// To EEPROM
void saveValueEEPROM(int mode, int infe, int trl1, int mrl2, int trl2, int mled, int gain);

// To converter String to char to EEPROM
void writeStringToEEPROM(char add, String data);
void readStringLikeChar(char data[]);

// To interface
void interfaceLED();
void interfaceRELAY();

// To MCP41010
void digitalPotWrite(int value);

// To serial
char processCharInput(char* cmdBuffer, const char c);

//*****************************************************************************************************************
// Object(s)
//*****************************************************************************************************************
      
//*****************************************************************************************************************
// Global variable(s)
//*****************************************************************************************************************               
// To operate
bool flagCalculateInference    = true;
bool flagSendJson              = true;
bool flagErroConnect           = true;
bool flagShotDetect            = false;
bool flagToTest                = false;
bool flagToConfigureTMDev      = false;
bool flagErrorData             = true;
        
// To keepalive       
int keepAliveCounter            = 0;
unsigned long keepAliveTime     = 0;
      
// To converter String to char      
// char SNvalue[MAX_SIZE_SN]        = "SNTESTE";
char SNvalue[MAX_SIZE_SN]         = "";
      
// To MCP41010      
byte address                = 0x11;

// To TFLite
// TFLite globals, used for compatibility with Arduino-style sketches
namespace 
{
  
  tflite::ErrorReporter* error_reporter   = nullptr;
  const tflite::Model* model            = nullptr;
  
  tflite::MicroInterpreter* interpreter   = nullptr;
  TfLiteTensor* model_input             = nullptr;
  TfLiteTensor* model_output            = nullptr;
  
  // Create an area of memory to use for input, output, and other TensorFlow
  // arrays. You'll need to adjust this by compiling, running, and looking
  // for error.
  constexpr int kTensorArenaSize  = 40000; // A modifier selon le modèle
  uint8_t tensor_arena[kTensorArenaSize];
   
} //namespace

// Variaveis para as Tasks    
int count                             = 0;
char buffer_full                      = 0x00;
short input_buffer_pointer_count        = 0;
short output_buffer_pointer_count       = 1;
// To buffer
int16_t samplesBuffer[frameShift]; // changé le perte d echantillons
double mfcc_output[256]; // 256?
float inference[16];
float sum_inference = 0;
//double vReal_janela[FFT_SIZE];                                    // Aloca memÃ³ria pra janela da FFT
//v_d mfcc_output_frame;                                          // Aloca memÃ³ria pra uma MFCC e a MAtriz MFCC
//double mfcc_output[100];
// To time
unsigned long timeNow                 = 0;
unsigned long timeAfter                 = 0;
unsigned long timeDifference            = 0;
// To MFCC
int window_index                    = 56;
unsigned short inference_index        = 0;
// To counter
int counter                       = 0;
//double vReal_janela[FFT_SIZE];                          
v_d mfcc_output_frame;                              

// To inference
//int valueInferenceNow;
float valueInferenceNow;

// To gain 
int valueGain;

// To event
char valueEvent               = 0x00;

unsigned long timeNow1            = 0;
unsigned long timeNow2            = 0;
unsigned long timeToJson01          = 0;
unsigned long timeToJson02          = 0;

// To verifier json
char flagVerifierJson           = 0x00;

// To timer in hardware
hw_timer_t * timer              = NULL;  
//hw_timer_t * timer0             = NULL;  
//hw_timer_t * timer1             = NULL;  
//hw_timer_t * timer2             = NULL;  
//          
//portMUX_TYPE timerMux0          = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE timerMux1          = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE timerMux2          = portMUX_INITIALIZER_UNLOCKED;
//        
//volatile uint8_t led1stat         = 0; 
//volatile uint8_t led2stat         = 0; 
//volatile uint8_t led3stat         = 0; 

// To verifier connect
// To commands AT
static char cmdBuffer[CMDBUFFER_SIZE]     = "";

// To interface
char flagTurnOnLED              = 0x00;
char flagTurnRelay01            = 0x00;
char flagTurnRelay02            = 0x00;
unsigned long timeToLED           = 0;
unsigned long timeToRelay01         = 0;
unsigned long timeToRelay02         = 0;
unsigned long timeToErrorConnect      = 0;
int timeToRelayFix01            = 0;
int timeToRelayFix02            = 0;

// To calibration
bool flagToCalibration            = false;
bool flagStartCalibration         = false;
int counterPushButton01           = 50;   
int counterPushButton           = 0;
unsigned long timeCalibrationNow      = 0;
float totalADC                = 0;
float voltageValueOffset          = 0;
float voltageValue              = 0;
float voltageRMS              = 0;  
int counterVoltage              = 0;
float Vrms                  = 0.0;                // Armazena o valor rms da tensÃ£o
float Vmax                  = 0.0;                // Armazena o valor mÃ¡ximo detectado
float Vmin                  = 10000.0;              // Armazena o valor mÃ­nimo detectado
float Vmed                  = 0.0;                // Armazena o valor mÃ©dio entre VmÃ¡x e VmÃ­n
const float fatorDivisor          = 168.40166345742404792461;     // Valor teÃ³rico divisor de tensÃ£o  = 168.85714285714285714286
const float fatorAmplificador         = 1.0;                // Valor teÃ³rico do ganho de amplificaÃ§Ã£o = 1.0
const float fatorMultiplicacao        = fatorDivisor*fatorAmplificador;   // Valor usado na multiplicaÃ§Ã£o da leitura
const float Vcc               = 3.3;                // Valor teÃ³rico da TensÃ£o de alimentaÃ§Ã£o Vcc = 3.3V
const float offSet              = 1.58;               // Valor teÃ³rico do offset do amplificador = Vcc / 2.0;
const float fatorAD             = Vcc/4095.0;           // Fator teÃ³rico da conversÃ£o do AD = 3.3 / 4095.0
const int amostras              = 71429;              // Resulta em 1,027 segundos para cada atualizaÃ§Ã£o
//const int amostras            = 35715;              // Resulta em 0,514 segundos para cada atualizaÃ§Ã£o
//const int amostras            = 35715;              // Resulta em 0,514 segundos para cada atualizaÃ§Ã£o
const float valueFix            = 0.150;
int valueGainRate             = 50;


// To I2S
int16_t buffer1[bufferLen]; // Buffer to store the 16-bit audio samples
bool flagI2S = false;

// To message Json to send
// Model 01:  MESSAGE DEFAULT
// Model 02:  MESSAGE OF THE CONNECT SUCESS 
// Model 03:  MESSAGE OF THE CONNECT FAIL 
// Model 04:  MESSAGE TO KEEPALIVE  
// ****************************************************************************************************************
// Model 01:  {"HEADER":"status","TYPE":"GSD",â€œVERSâ€:â€x.yyâ€,"SN":"xxxxxxx",â€œEVNTâ€:x}
// Model 02:  {"HEADER":"connectSucess","SN":"xxxxxxx","MODE":x,"INFE":x,"TRL1":x,"MRL2":x,"TRL2":x,"MLED":x,"GAIN":x}
// Model 02:  {"HEADER":"resetSucess","SN":"xxxxxxx","MODE":x,"INFE":x,"MRL1":x,"TRL1":x,"MRL2":x,"TRL2":x,"MLED":x,"GAIN":x}
// Model 03:  {"HEADER":"connectFail","SN":"xxxxxxx"}
// Model 04:  {"HEADER":"keepalive"}

// To message Json to receiveJson
// Model 01:  MESSAGE TO TRY CONNECT
// Model 02:  MESSAGE TO CONFIG.
// ****************************************************************************************************************
// Model 01:  {"HEADER":"connect","SN":"xxxxxxx","PW":"s3nh4@mtwgsd"}
// Model 01.1:  {"HEADER":"connect","SN":"SNTESTE","PW":"s3nh4@mtwgsd"}
// Model 01.2:  {"HEADER":"connect","SN":"Sm8MhFi","PW":"s3nh4@mtwgsd"}
// Model 01.3:  A55AA5{"HEADER":"connect","SN":"r71T94S","PW":"s3nh4@mtwgsd"}
// Model 02:  {"HEADER":"config","SN":"xxxxxxx","MODE":x,"INFE":x,"TRL1":x,"MRL2":x,"TRL2":x,"MLED":x,"GAIN":x}
// Model 02.1:  {"HEADER":"config","SN":"1515150","MODE":0,"INFE":5,"TRL1":500,"MRL2":1,"TRL2":500,"MLED":1,"GAIN":5}
// Model 02.2:  {"HEADER":"config","SN":"1515150","MODE":1,"INFE":4,"TRL1":500,"MRL2":2,"TRL2":500,"MLED":1,"GAIN":1}
// Model 02.3:  {"HEADER":"factoryReset","MRL1":0} //Muda o serial number igual a SNTESTE e desabilita o estado do alarme.
// Model 02.4:  {"HEADER":"factoryReset","MRL1":1} //Muda o serial number igual a SNTESTE e habilita o estado do alarme.

//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
// Initial interrupt
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************

//#ifdef  DEBUG_INTERRUPT
//void IRAM_ATTR interruptGSD20_01() 
//{
//
//  flagToTest          = true;
//  
//} // end interruptGSD20_01
//#endif

void IRAM_ATTR interruptGSD20_02() 
{

  flagToConfigureTMDev    = true;
  
} // end interruptGSD20_02

void IRAM_ATTR interruptGSD20_03() 
{

  flagToConfigureTMDev    = false;
  
} // end interruptGSD20_03

/*

void IRAM_ATTR onTimer0()
{
  
  // Critical Code here
  portENTER_CRITICAL_ISR(&timerMux0);
  led1stat=1-led1stat;
  digitalWrite(16, led1stat);   // turn the LED on or off
  portEXIT_CRITICAL_ISR(&timerMux0);

} // end IRAM_ATTR onTimer0

void IRAM_ATTR onTimer1()
{
  
  // Critical Code here
  portENTER_CRITICAL_ISR(&timerMux1);
  led2stat=1-led2stat;
  digitalWrite(17, led2stat);   // turn the LED on or off
  portEXIT_CRITICAL_ISR(&timerMux1);
  
} // end IRAM_ATTR onTimer1

void IRAM_ATTR onTimer2()
{
  
  // Critical Code here
  portENTER_CRITICAL_ISR(&timerMux2);
  led3stat=1-led3stat;
  digitalWrite(5, led3stat);   // turn the LED on or off
  portEXIT_CRITICAL_ISR(&timerMux2);

} // end IRAM_ATTR onTimer2

*/

//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
// End settings
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************

//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
// Initial settings
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************

void setup()
{
  setupSerial();
  delay(3000);
  Serial.println("Start Up");
  
  // To clear EEPROM
  
  /*
  while(1)
  {

    EEPROM.begin(MAX_EEPROM);

    delay(5000*timeToWait);
    
    //for (int i = 0; i < EEPROM.length(); i++) 
    for (int i = 0; i < MAX_EEPROM; i++)  
    {
      
      Serial.println("Read EEPROM before clear! "); 
      delay(500*timeToWait);
      Serial.println(EEPROM.read(i)); 
    
    } // end for  

    //for (int i = 0; i < EEPROM.length(); i++) 
    for (int j = 0; j < MAX_EEPROM; j++)  
    {
      
      Serial.println("Clear EEPROM! "); 
      delay(500*timeToWait);
      EEPROM.write(j, 0x00);  
      
      EEPROM.commit();
      
    } // end for  
    
    //for (int  i= 0; i < EEPROM.length(); i++) 
    for (int k = 0; k < MAX_EEPROM; k++)  
    {
      
      Serial.println("Read EEPROM after clear! ");
      delay(500*timeToWait);      
      Serial.println(EEPROM.read(k)); 
    
    } // end for
    
    while(1)  Serial.println("Ok"); 
    
  } // end while  
  */
  
  setupPins();
  Serial.println("Setup pins done");
  setupMCP41010();  
  Serial.println("Setup MCP41010 done");
  setupEEPROM();  
  Serial.println("Setup EEPROM done");
  // if(EEPROM.read(modeCalibration)  ==  0x00) setupCalibration();
  // setupCalibration();
  Serial.println("Setup Calibration done"); // !!
  Serial.println("No Calibration ADC");
  setupMFCC();
  Serial.println("Setup MFCC done");
  setupTFLite();
  Serial.println("Setup TFLite done");
  setupI2S();
  Serial.println("Setup I2S done");
  setupTimer(); 
  Serial.println("Setup Timer done");
  setupInterrupt();
  Serial.println("Setup Interrupt done");
  timeToJson01 = millis();

  Serial.println("Setup done");

} // end setup

//*****************************************************************************************************************
// Setup serial
//*****************************************************************************************************************

void setupSerial()
{
  
  Serial.begin(SERIAL);
  Serial1.begin(SERIAL, SERIAL_8N1, pinToRX, pinToTX);  
  Serial2.begin(SERIAL);

} // end setupSerial

//*****************************************************************************************************************
// Setup pins
//*****************************************************************************************************************

void setupPins()
{ 
  
  // To interface
  pinMode(pinToLED,     OUTPUT);
  pinMode(pinToRelay01,   OUTPUT);
  pinMode(pinToRelay02,   OUTPUT);
  pinMode(pinToCalibration, INPUT_PULLUP);

  // To ADC
  pinMode(pinReadADC01,     INPUT);                     // Pin to ADC 01. 
  pinMode(pinReadADC02,     INPUT);                     // Pin to ADC 02. 

  // To DAC
  /*
  pinMode(pinWriteDAC01, OUTPUT);                         // Pin to DAC01.
  pinMode(pinWriteDAC02, OUTPUT);                         // Pin to DAC02.
  
  dacWrite(pinWriteDAC01, 0xff);
  dacWrite(pinWriteDAC01, 0x00);
  
  dacWrite(pinWriteDAC02, 0xff);
  dacWrite(pinWriteDAC02, 0x00);

  */

  digitalWrite(pinToLED,    HIGH);
  //digitalWrite(pinToRelay01,  HIGH);
  //digitalWrite(pinToRelay02,  HIGH);
  delay(timeDefaultRelays*timeToWait);
  digitalWrite(pinToLED,    LOW);
  //digitalWrite(pinToRelay01,  LOW);
  //digitalWrite(pinToRelay02,  LOW);
  delay(timeDefaultRelays*timeToWait);  
  
  //digitalWrite(pinToRelay01,  HIGH);
  //if    (EEPROM.read(relay02Mode) == 0x01)  digitalWrite(pinToRelay02,  LOW);
  //else if (EEPROM.read(relay02Mode) == 0x02)  digitalWrite(pinToRelay02,  HIGH);
  //else                    digitalWrite(pinToRelay02,  HIGH);
  
} // end setupPins

//*****************************************************************************************************************
// Setup MCP41010
//*****************************************************************************************************************

void setupMCP41010()
{
  
  if(EEPROM.read(flagEEPROM)  == 0x01)
  {
    
    pinMode(pinCS1,       OUTPUT);
    pinMode(pinCS2,       OUTPUT);
    pinMode(pinCS3,       OUTPUT);
    SPI.begin();
    
  } // end if
  
  else
  {

    pinMode(pinCS1,       OUTPUT);
    pinMode(pinCS2,       OUTPUT);
    pinMode(pinCS3,       OUTPUT);
    SPI.begin();
    
    // Representa 1k, que no circuito Ã© ganho 1.
    // digitalPotWrite(MIN_VALUE_MCP41010);
    digitalPotWrite(25);

  } // end else

} // end setupMCP41010

//*****************************************************************************************************************
// Setup EEPROM
//*****************************************************************************************************************

void setupEEPROM()
{ 

  EEPROM.begin(MAX_EEPROM);
    
  if(EEPROM.read(flagEEPROM)  == 0x01)
  {

    digitalWrite(pinToLED,    HIGH);
    //digitalWrite(pinToRelay01,  HIGH);
    //digitalWrite(pinToRelay02,  HIGH);
    delay(timeDefaultRelays*timeToWait);
    digitalWrite(pinToLED,    LOW);
    //digitalWrite(pinToRelay01,  LOW);
    //digitalWrite(pinToRelay02,  LOW);
    delay(timeDefaultRelays*timeToWait);
    
    valueInferenceNow = EEPROM.read(inferenceValues);
    valueGain     = EEPROM.read(gainAmpOp);
    valueEvent      = 0x00;
    timeToRelayFix01  =   (EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1));
    timeToRelayFix02  =   (EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1));
    
    valueGain     = (valueGain*FIX_NORMALIZE_VARIABLE);
    valueInferenceNow =   ((valueInferenceNow*FIX_MAX_INFE)/(MAX_RATE));
    
    if(valueGain      < MIN_VALUE_MCP41010) valueGain     = MIN_VALUE_MCP41010;
    if(valueInferenceNow  < MIN_VALUE_INFE)   valueInferenceNow = MIN_VALUE_INFE;
    
    digitalPotWrite(valueGain);
    
    interfaceRELAY();
  
  } // end if   
  
  else
  {
  
    EEPROM.write(modeOperation,       0x00);
    EEPROM.write(inferenceValues,       0x04);
    EEPROM.write(relay01Mode,         0x00);
    EEPROM.write(timeRelay01Mode,       timeDefaultRelays/256);
    EEPROM.write(timeRelay01Mode+1,     timeDefaultRelays%256);
    EEPROM.write(relay02Mode,         0x02);
    EEPROM.write(timeRelay02Mode,       timeDefaultRelays/256);
    EEPROM.write(timeRelay02Mode+1,     timeDefaultRelays%256);
    EEPROM.write(ledOperation,        0x01);
    EEPROM.write(gainAmpOp,         0x01);          
    EEPROM.write(eventGSD,          0x00);      
    writeStringToEEPROM(serialNumberFinalGSD20, serialNumberTestGSD20);
    EEPROM.write(flagEEPROM,        0x01);
    
    EEPROM.commit();
    
    valueInferenceNow = EEPROM.read(inferenceValues);
    valueGain     = EEPROM.read(gainAmpOp);
    valueEvent      = 0x00;
    timeToRelayFix01  =   (EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1));
    timeToRelayFix02  =   (EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1));  

    digitalPotWrite(valueGain);   
    
    digitalWrite(pinToLED,    HIGH);
    //digitalWrite(pinToRelay01,  HIGH);
    //digitalWrite(pinToRelay02,  HIGH);
    delay(timeDefaultRelays*timeToWait);
    digitalWrite(pinToLED,    LOW);
    //digitalWrite(pinToRelay01,  LOW);
    //digitalWrite(pinToRelay02,  LOW);
    delay(timeDefaultRelays*timeToWait);  
    digitalWrite(pinToLED,    HIGH);
    //digitalWrite(pinToRelay01,  HIGH);
    //digitalWrite(pinToRelay02,  HIGH);
    delay(timeDefaultRelays*timeToWait);
    digitalWrite(pinToLED,    LOW);
    //digitalWrite(pinToRelay01,  LOW);
    //digitalWrite(pinToRelay02,  LOW);   
  
  } // end if
  
  interfaceRELAY();
  
} // end setupJson

//*****************************************************************************************************************
// Setup MFCC
//*****************************************************************************************************************

void setupMFCC()
{
  // revoir par rapport la noivelle biblioteque
  MFCC_INIT(FFT_SIZE, samplingRate, numCepstra, frameShift, numFilters, lowFreq, highFreq);
  
} // end setupMFCC

//*****************************************************************************************************************
// Setup TFLite
//*****************************************************************************************************************

void setupTFLite()
{
  
  // Set up logging (will report to Serial, even within TFLite functions)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(all_targets_tflite);
  // pointer vers le tien rna
  
  if (model->version() != TFLITE_SCHEMA_VERSION) 
  {
    
    error_reporter->Report("Model version does not match Schema");
    while(1);
  
  } // end if
  
  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::ops::micro::AllOpsResolver resolver;
  
  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
                model, 
                resolver, 
                tensor_arena, 
                kTensorArenaSize,
                error_reporter);
                
  interpreter = &static_interpreter;
  
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  
  if (allocate_status != kTfLiteOk) 
  {
    
    error_reporter->Report("AllocateTensors() failed");
    while(1);
  
  } // end if
  
  // Obtain pointers to the model's input and output tensors.
  model_input   = interpreter->input(0);
  model_output  = interpreter->output(0);
  
  // Get information about the memory area to use for the model's input
  /*#if DEBUG
  Serial.print("Number of dimensions: ");
  Serial.println(model_input->dims->size);
  Serial.print("Dim 1 size: ");
  Serial.println(model_input->dims->data[0]);
  Serial.print("Dim 2 size: ");
  Serial.println(model_input->dims->data[1]);
  Serial.print("Dim 3 size: ");
  Serial.println(model_input->dims->data[2]);
  Serial.print("Dim 4 size: ");
  Serial.println(model_input->dims->data[3]);
  Serial.print("Input type: ");
  Serial.println(model_input->type);
  #endif*/  
  
} // end setupTFLite

//*****************************************************************************************************************
// Setup timer
//*****************************************************************************************************************

void setupTimer()
{
  Serial.println("Setup Timer");
  /* 
  InicializaÃ§Ã£o do timer. Parametros:
  0 - seleÃ§Ã£o do timer a ser usado, de 0 a 3.
  80 - prescaler. O clock principal do ESP32 Ã© 80MHz. Dividimos por 80 para ter 1us por tick.
  true - true para contador progressivo, false para regressivo
  */
  timer = timerBegin(0, 80, true);
  
  Serial.println("Start Interruption Timer");
  /*
  Conecta Ã  interrupÃ§Ã£o do timer
  - timer Ã© a instÃ¢ncia do hw_timer
  - endereÃ§o da funÃ§Ã£o a ser chamada pelo timer
  - edge = true gera uma interrupÃ§Ã£o
  */
  timerAttachInterrupt(timer, &timerI2S, true); // pointer veers la i2S
  
  Serial.println("Setup Alarm Timer");
  /* 
  - o timer instanciado no inicio
  - o valor em us para 1s
  - auto-reload. true para repetir o alarme
  */
  timerAlarmWrite(timer, 125, true); 
  
  Serial.println("Enable Alarm Timer");
  // Ativa o alarme
  timerAlarmEnable(timer);
  
} // end setupTimer

//*****************************************************************************************************************
// Disable timer
//*****************************************************************************************************************

void disableTimer()
{
  
  // Disable timer
    timerAlarmDisable(timer);                         // Stop alarm
  
  // Detach interrupt
    timerDetachInterrupt(timer);                        // Detach interrupt
  
  // End timer
  timerEnd(timer);                              // End timer  
  
} // end disableTimer

//*****************************************************************************************************************
// Interrupt to serial
//*****************************************************************************************************************

void setupInterrupt()
{
  
  //#ifdef  DEBUG_INTERRUPT
  //attachInterrupt(digitalPinToInterrupt(pinToInterrupt01), interruptGSD20_01, CHANGE);
  //#endif
  
  attachInterrupt(digitalPinToInterrupt(pinToInterrupt02), interruptGSD20_02, RISING);
  
} // end setupInterrupt

//*****************************************************************************************************************
// Setup calibration
//*****************************************************************************************************************

/*
void setupCalibration()
{
  
  flagToCalibration = true;
  digitalWrite(pinToLED,  HIGH);
  
  // Get the value offset
  for(int i = 0; i < 5; i++)
  {
  
    totalADC        = 0;
    voltageValueOffset    = 0;
    voltageValue      = 0;
    voltageRMS        = 0;
    counterVoltage      = 0;
    timeCalibrationNow    = 0;  
  
    timeCalibrationNow  = millis();   
    while(millis()  - timeCalibrationNow  < 5000) 
    {
      
      counterVoltage++;
      totalADC += 1.0 * analogRead(pinReadADC01);
      
    } // end while
        
    totalADC      = totalADC/counterVoltage;    
    voltageValueOffset  = (totalADC * 3.3)/4096.0;
    voltageValue    = (voltageValueOffset-1.65);
    voltageRMS      = voltageValue*1.1107;  
    
    #ifdef  DEBUG_CALIBRATION
    Serial.print("Value counter is: ");
    Serial.println(counterVoltage);
    Serial.print("Value ADC is: ");
    Serial.println(totalADC);
    Serial.print("Value voltage offset is: ");
    Serial.println(voltageValueOffset);       
    //Serial.print("Value voltage is: ");
    //Serial.println(voltageValue);
    //Serial.print("Value RMS is: ");
    //Serial.println(voltageRMS);
    #endif

  } // end for
  
  pinMode(pinCS1,       OUTPUT);
  pinMode(pinCS2,       OUTPUT);
  pinMode(pinCS3,       OUTPUT);
  SPI.begin();  
  digitalPotWrite(valueGainRate);
  
  while(flagToCalibration)
  {
    
    int i       = 0;                          // VariÃ¡vel para iteraÃ§Ã£o
    float leitura   = 0.0;                          // Armazena as leituras do AD
    Vrms      = 0.0;                          // Reinicia a variÃ¡vel Vrms
    Vmax      = 0.0;
    //Vmin      = 0.0;
    
    // Inicia um ciclo de amostragem atÃ© que i alcance o nÃºmero de amostras
    while (i < amostras)                            
    { 
    
      leitura = analogRead(pinReadADC01);                 // lÃª a porta analÃ³gica
      //Serial.println(leitura);                      // Descomente se quiser ver o sinal bruto do AD
      Vrms  = Vrms + pow(((leitura * fatorAD) - voltageValueOffset), 2.0);      // calcula a soma dos quadrados das tensÃµes lidas
      // Vrms   = Vrms + pow(((leitura * fatorAD) - offSet), 2.0);      // calcula a soma dos quadrados das tensÃµes lidas
      i++;                                // Incrementa o iterador
    
    } // end while

    // Vrms = (sqrt(Vrms / amostras)) * fatorMultiplicacao;         // Aplicando fator de multiplicaÃ§Ã£o para determinar o valor real das tensÃµes
    Vrms = (sqrt(Vrms / amostras));                     // Aplicando fator de multiplicaÃ§Ã£o para determinar o valor real das tensÃµes
    
    if (Vrms > Vmax)  Vmax = Vrms;                    // Detecta se Ã© um valor Ã© mÃ¡ximo
    if (Vrms < Vmin)  Vmin = Vrms;                    // Detecta se Ã© um valor mÃ­nimo
    
    Vmed = (Vmax + Vmin)/2.0;
  
    #ifdef  DEBUG_CALIBRATION
    Serial.print("Vrms: ");
    Serial.println(Vrms, 3);
    //Serial.print(",");
    //Serial.print(Vmax, 3);
    //Serial.print(",");
    //Serial.print(Vmin, 3);
    //Serial.print(",");
    //Serial.println(Vmed, 3);  
    #endif    
    
    //if(Vrms !=  valueFix)
    //{
      
    float valueRate   = (valueFix/Vrms);
    Serial.print("Value rate: ");
    Serial.println(valueRate, 3);
    
    valueGainRate   = (valueGainRate*valueRate);
    Serial.print("Value gain rate after: ");
    Serial.println(valueGainRate);  

    if    (valueGainRate  > 250)  valueGainRate = 250;
    else if (valueGainRate  <=  0)    valueGain   = 1;
    
    Serial.print("Value gain rate before: ");
    Serial.println(valueGainRate);  

    digitalPotWrite(valueGainRate);
    delay(1000*timeToWait);
      
    //} // end if
    
  } // end while
  
  digitalWrite(pinToLED,  LOW);
  
} // end setupCalibration
*/

//*****************************************************************************************************************
// Setup I2S
//*****************************************************************************************************************

void setupI2S()
{
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),                       // I2S mode: Master, Receive
    .sample_rate = samplingRate,                                             // Sample rate
    .bits_per_sample = i2s_bits_per_sample_t(16),                            // Bits per sample
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                             // Channel format: Only left channel
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),    // Communication format: Standard I2S
    .intr_alloc_flags = 0,                                                   // Interrupt allocation flags
    .dma_buf_count = 8,                                                      // DMA buffer count
    .dma_buf_len = bufferLen,                                                // Length of each DMA buffer
    .use_apll = false                                                        // Use APLL for master clock
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);   // Install and configure the I2S driver

  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,                // Serial Clock pin
    .ws_io_num = I2S_WS,                  // Word Select pin
    .data_out_num = -1,                   // Data output pin (not used in this case)
    .data_in_num = I2S_SD                 // Serial Data pin
  };

  i2s_set_pin(I2S_PORT, &pin_config);     // Set the I2S pin configuration

  i2s_start(I2S_PORT);                    // Start the I2S driver
}

//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
// End settings
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************
//*****************************************************************************************************************

//*****************************************************************************************************************
// Loop infinite
//*****************************************************************************************************************

void loop()
{
  // Calibration
  /*
  if(!digitalRead(pinToCalibration))    flagToCalibration = true;
  if((!digitalRead(pinToCalibration)) &&  flagToCalibration)
  {
    
    while(!digitalRead(pinToCalibration))
    {
      
      digitalWrite(pinToLED,    HIGH);
      
      counterPushButton++;
      Serial.print("COUNTER TO PUSH BUTTON: ");   
      Serial.println(counterPushButton);   
      delay(100*timeToWait);
        
    } // end while
    
    if    (counterPushButton  >=  counterPushButton01)  
    {
      
      digitalWrite(pinToLED,    LOW);
      //counterPushButton = 0;
      //flagToCalibration = false;
      
      EEPROM.write(modeCalibration, 0x01);
      EEPROM.commit();
      
      delay(3000*timeToWait);
      
      ESP.restart();
      
    } // end if   
    
  } // end if  // Peut etre commenter cette calibration pour le moment
  */
  // End Calibration 

  if(flagCalculateInference) calculateInference();
    
  if(!flagErroConnect)
  {
    
    if(millis() - timeToErrorConnect >  timeToBlock)  flagErroConnect = true;
    
  } // end if
  

  if((flagSendJson) && (flagErroConnect))       
  //if((flagSendJson))        
  {
        
    if(millis() - timeToJson01 > timeToJsonWait01)
    { 
        
      if(EEPROM.read(modeOperation) == 0x01)
      {
      
        sendJsonDefaut();
      
        if(flagToConfigureTMDev)
        {
          
          attachInterrupt(digitalPinToInterrupt(pinToInterrupt02), interruptGSD20_03, FALLING);
          
          //timeToJson02  = millis();
          
          //while(millis() - timeToJson02 < timeToJsonWait02)
          //
          
          sendJsonDefaut();
          if((Serial1.available() > 0)) verifierJsonConnect();
            
          // end while

          //flagToConfigureTMDev  = false;
          //timeToJson02      = 0;
          
        } // end if 
      
      } // end if
      
      else
      {
        
        sendJsonDefaut();
        
        if((Serial.available() > 0))  verifierJsonConnect();  
        
      } // end else
            
      timeToJson01  = millis(); 
          
    } // end if   
                  
  } // end if 


} // end loop

//*****************************************************************************************************************
// Process char input
//*****************************************************************************************************************

char processCharInput(char* cmdBuffer, const char c)
{
  
  // Store the character in the input buffer
  // Ignore control characters and special ascii characters
  if (c >= 32 && c <= 126) 
  {
    
    if (strlen(cmdBuffer) < CMDBUFFER_SIZE) strncat(cmdBuffer, &c, 1);   //Add it to the buffer
    else                  return '\n';
  
  } // end if
  
  //Backspace
  else if ((c == 8 || c == 127) && cmdBuffer[0] != 0) cmdBuffer[strlen(cmdBuffer)-1] = 0;
  
  return c;

} // end processCharInput

//*****************************************************************************************************************
// Calculate inference
//*****************************************************************************************************************

void calculateInference()
{
  if(flagI2S)
  { 
    // Serial.println("I2S");
    // if(Serial.available())
    //   {
        samplesBuffer[counter] = Serial.parseInt();
        // Serial.printf("Serial[%d]: %d\n", counter, samplesBuffer[counter]);
        counter++;
      // }
    flagI2S = false;
  }
  
  if(buffer_full == 0x01)
  { 
    
    //#ifdef  DEBUG_INTERRUPT
    //if(flagToTest)  
    //{
    //  
    //  sum_inference = 5;      
    //  
    //} // end if
    //#endif
  
    mfcc_output_frame = mfcc_processFrame(samplesBuffer, frameShift, FFT_SIZE, numFilters, numCepstra);
  
    for (int output_index = 0; output_index < 16; output_index++)
    {
      mfcc_output[window_index+output_index] = mfcc_output_frame[output_index];
    } // end for
    
    window_index += 16;// decalage de 16
  
    if (window_index >= 255)     //255                      // apÃ³s calcular 4 MFCC (0,5 segundos e pega as MFCCs de trÃ¡s)
    {
      
      //for (short i=0; i<63; i++)
      //Serial.println(mfcc_output[i]);
      // Jogar a MFCC pro ponteiro do TFLite
      for(short output_buffer_index=0; output_buffer_index < 255; output_buffer_index++)
      {
        model_input -> data.f[output_buffer_index] = mfcc_output[output_buffer_index]; 
      } // end for
      
      // Run inference
      TfLiteStatus invoke_status = interpreter->Invoke();
    
      if (invoke_status != kTfLiteOk) error_reporter->Report("Invoke failed on input: %f\n", mfcc_output[0]);
    
      // Read predicted y value from output buffer (tensor)
      float inf_1 = model_output->data.f[0];
      Serial.printf("Prediction : %f\n", inf_1);

      inference[inference_index]=inf_1;
  
      if (inference_index == 15) inference_index = 0;
      else inference_index++;
  
      for (unsigned short ind = 0; ind < 16; ind++)
      {
        
        sum_inference = sum_inference + inference[ind];
              
        // Serial.print("Sum Inference 1 is: ");     // DEBUG Clement
        // Serial1.print("Sum Inference 1 is: ");
        // Serial2.print("Sum Inference 1 is: ");
        // Serial.println(sum_inference);
        // Serial1.println(sum_inference);
        // Serial2.println(sum_inference);

  
      } // end for
  
      if(flagShotDetect)
      {

        //Serial.println("Estou aqui 01! ");

        //************************
        // To LED
        //************************  
        /*
        if(millis() - timeToLED   > timeToLEDFix) 
        {
          
          flagTurnOnLED = 0x00;
          interfaceLED();
          if( (flagTurnRelay01 == 0x01) ||  
            (flagTurnRelay02 == 0x01) ||
            (flagTurnRelay02 == 0x02) ) flagShotDetect  = true;
            
          else                flagShotDetect  =   false;
          
        } // end if 
        */
        
        //************************
        // To Relay 01
        //************************  
        if(millis() - timeToRelay01 > timeToRelayFix01) 
        {
          
          flagTurnRelay01 = 0x00;
          interfaceRELAY();
          flagShotDetect  =   false;
          
        } // end if 
        
        //************************
        // To Relay 02
        //************************      
        if(millis() - timeToRelay02 > timeToRelayFix02) 
        {
          
          flagTurnRelay02 = 0x00;
          interfaceRELAY();
          flagShotDetect  =   false;
          
        } // end if   
        
      } // end if 
    
      if (sum_inference > valueInferenceNow)
      {
      
        disableTimer();
        
        //Serial.println("Estou aqui 02! ");
        
        sum_inference   = 0;
        flagToTest      = false;
        flagShotDetect    = true;
        
        //************************
        // To LED
        //************************
        if(EEPROM.read(ledOperation)  == 0x01)
        {
          
          flagTurnOnLED = 0x01; 
          interfaceLED();
          delay(100*timeToWait);
          flagTurnOnLED = 0x00; 
          interfaceLED();
          //timeToLED   = millis();
          
        } // end if 
  
        //************************
        // To Relay 01
        //************************
  
        if    (EEPROM.read(relay01Mode) == 0x01)
        {
          
          flagTurnRelay01 = 0x01;
          interfaceRELAY();
          timeToRelay01 = millis();
        
        } // end if 
  
        //************************
        // To Relay 02
        //************************      
        if    ((EEPROM.read(relay02Mode)  == 0x01)  &&  ((EEPROM.read(relay01Mode)  == 0x01)))
        {
          
          flagTurnRelay02 = 0x01;
          interfaceRELAY();
          timeToRelay02 = millis();
        
        } // end if
        
        else if (EEPROM.read(relay02Mode) == 0x02)
        {
          
          flagTurnRelay02 = 0x02;
          interfaceRELAY();
          timeToRelay02 = millis();
        
        } // end if
        
        else
        {
          
          flagTurnRelay02 = 0x00;
          interfaceRELAY();
          timeToRelay02 = millis(); 
        
        } // end if
        
        valueEvent  = 0x01;
        //sendJsonDefaut();
        
        //Serial.print("TIRO: ");
        //Serial1.print("TIRO: ");
        //Serial2.print("TIRO: ");
        //Serial.println(sum_inference);
        //Serial1.println(sum_inference);
        //Serial2.println(sum_inference);
      
        setupTimer();
      
      } // end if
          
      else
      {
        
        valueEvent  = 0x00;

      } // end else
  
      sum_inference = 0;
  
      for(short i = 16; i < 256; i++)
      {
        mfcc_output[i-16] = mfcc_output[i];
      }

      window_index = 239; // 255 - 16
    }
        
    /* 
    for (short array_copy_index = 0; array_copy_index < FFT_SIZE; array_copy_index++)
    {
        
      vReal_janela[array_copy_index] = (double) (now_buffer[array_copy_index+mfcc_index])/10;
    
    } // end for
  
  
    for(short mfcc_index = 0; mfcc_index < nSamplesBuff; mfcc_index += mfcc_step)
    {
    
      for (short array_copy_index = 0; array_copy_index < FFT_SIZE; array_copy_index++)
      {
      
        vReal_janela[array_copy_index] = (double) (now_buffer[array_copy_index+mfcc_index])/10;
      
      } // end for
    
      // mfcc_output.assign(8,15);
      
      for (int output_index = 0; output_index < 8; output_index++)
      {
        
        mfcc_output[window_index+output_index] = mfcc_output_frame[output_index];
      
      } // end for
      
      window_index += 8;
    
      //Arrumar aqui que continua aumentando o tamanho do vetor indefinidamente    
    } // end for
      
    window_index = 0;
    */
  
    /*
    for(int i = 0; i < nSamplesBuff; i++)
    {
      
      now_buffer[i] = samplesBuffer_0[i];
    
    } // end for
  
    for(int i = 0; i < nSamplesBuff; i++)
    {
      
      now_buffer[i+4000] = samplesBuffer_1[i];
    
    } // end for
  
    if (output_buffer_pointer_count == 1) 
    { 
      
      for(int i = 0; i < nSamplesBuff; i++)
      {
    
        now_buffer[i] = samplesBuffer_1[i];
    
      } // end for
  
      for(int i = 0; i < nSamplesBuff; i++)
      {
        
        now_buffer[i+4000] = samplesBuffer_2[i];
      
      } // end for
    
    } // end if 
  
  
    if (output_buffer_pointer_count == 2) 
    { 
    
      for(int i = 0; i < nSamplesBuff; i++)
      {
    
        now_buffer[i] = samplesBuffer_2[i];
      
      } // end for
  
      for(int i = 0; i < nSamplesBuff; i++)
      {
      
        now_buffer[i+4000] = samplesBuffer_0[i];
      
      } // end for
   
    } // end if 
    
    // INFERÃŠNCIA
    // Jogar a MFCC pro ponteiro do TFLite
    for(short output_buffer_index=16; output_buffer_index < 81; output_buffer_index++)
    {
    
      model_input -> data.f[output_buffer_index-16] = mfcc_output[output_buffer_index];
      //Serial.println(mfcc_output[output_buffer_index]);
    
    } // end for
    
    // Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    
    if (invoke_status != kTfLiteOk) error_reporter->Report("Invoke failed on input: %f\n", mfcc_output[0]);
    
    // Read predicted y value from output buffer (tensor)
    float inf_1 = model_output->data.f[0];
  
    Serial.print("Inference 1 is: ");
    Serial.println(inf_1);
  
    if (output_buffer_pointer_count > 1) output_buffer_pointer_count=0;
    e lse output_buffer_pointer_count++;*/
    
    buffer_full = 0x00;
    
  } // end if
    
} // end calculateInference

//*****************************************************************************************************************
// Function to timer
//*****************************************************************************************************************

void timerI2S()
{ 
  if((counter < FFT_SIZE))
  {
    // I2S read
    /*
    size_t bytesIn = 0;
    i2s_read(I2S_PORT, buffer1, bufferLen*2, &bytesIn, portMAX_DELAY);

    for(int i = 0; i < bytesIn; i++)
    {
      samplesBuffer[counter] = buffer1[i];
      counter++;
    } // end for
    */
    
    // Serial read
    // if(Serial.available())
    // {
    //   samplesBuffer[counter] = Serial.parseInt();
    //   Serial.printf("Serial: %d\n", samplesBuffer[counter]);
    //   counter++;
    // }

    flagI2S = true;

  } // end if
  else
  {
    counter = 0;  
    buffer_full = 0x01;
  } // end else
  
} // end timerI2S




//*****************************************************************************************************************
// Send json default
//*****************************************************************************************************************

void sendJsonDefaut()
{
  
  char msg[MAX_SIZE_JSON];
  
  StaticJsonDocument<MAX_SIZE_JSON> responseDefaultDoc;
  
  responseDefaultDoc[headerGSD20]     = status00GSD20;
  responseDefaultDoc[typeNameGSD20]     = typeGSD20;
  responseDefaultDoc[versionGSD20]    = firtVersionGSD20;
  readStringLikeChar(SNvalue);
  responseDefaultDoc[serialNumberGSD20]   = SNvalue;
  //responseDefaultDoc[detectEvent]     = EEPROM.read(eventGSD);
  responseDefaultDoc[detectEvent]     = String(valueEvent);
  
  serializeJson(responseDefaultDoc, msg);
    
  if(EEPROM.read(modeOperation) == 0x01)  
  {
    
    // Serial2.write(msg);      // UNCOMMENT !
    // Serial2.write("\n");
    
  } // end if
  
  else
  {
    
    // Serial.write(msg);       // UNCOMMENT !
    // Serial.write("\n");   
  
  } // end else
  
} // end sendJsonDefaut

//*****************************************************************************************************************
// Verifier json connect
//*****************************************************************************************************************

void verifierJsonConnect()
{
  
  disableTimer(); 
      
  StaticJsonDocument<MAX_SIZE_JSON> connectDoc; 
    
  if(EEPROM.read(modeOperation) == 0x01)  
  { 
  
    //DeserializationError error = deserializeJson(connectDoc, Serial2);  
    DeserializationError error = deserializeJson(connectDoc, Serial1);  
    flagVerifierJson = 0x01;
    
  } // end if 
      
  else                    
  {
    
    DeserializationError error = deserializeJson(connectDoc, Serial); 
    flagVerifierJson = 0x01;
    
  } // end else 
  
  if(flagVerifierJson == 0x01)
  {
  
    // Verifier HEADER
    if(connectDoc.containsKey(headerGSD20))
    {
      
      // Verifier connect
      if (strcmp(connectDoc[headerGSD20], status01GSD20) == 0)
      {
                  
        // Verifier PW
        if (connectDoc.containsKey(passwordGSD20))
        {
          
          // Verifier password
          if (strcmp(connectDoc[passwordGSD20], passwordTestGSD20) == 0)
          {
            
            // Verifier SN
            if(connectDoc.containsKey(serialNumberGSD20))
            {
            
              readStringLikeChar(SNvalue);
              // Verifier SNTESTE or other
              if    (strcmp(SNvalue, serialNumberTestGSD20) == 0) writeStringToEEPROM(serialNumberFinalGSD20, connectDoc[serialNumberGSD20]);
              
              else if (strcmp(connectDoc[serialNumberGSD20], SNvalue) == 0)
              {
                
                flagCalculateInference  = false;
                flagSendJson      = false;
                                  
                sendJsonConnect();
                verifierKeepAlive();  
                
              } // end else if
            
              else    
              {
                
                sendJsonFailConnect();  
                flagErroConnect   = false;  
                timeToErrorConnect  = millis();
                
              } // end else 
            
            } // end if
            
            else      
            {
              
              sendJsonFailConnect();  
              flagErroConnect   = false;  
              timeToErrorConnect  = millis();             
              
            } 
                        
          } // end if
          
          else        
          { 
          
            sendJsonFailConnect();  
            flagErroConnect   = false;    
            timeToErrorConnect  = millis();
        
          } // end else
          
        } // end if
        
        else          
        {
          
          sendJsonFailConnect();
          flagErroConnect   = false;  
          timeToErrorConnect  = millis();
          
        } // end else 
    
      } // end if
      
      else            
      {
        
        sendJsonFailConnect();
        flagErroConnect   = false;
        timeToErrorConnect  = millis();
        
      } // end else 
  
    } // end if
    
    else              
    {
      
      sendJsonFailConnect();  
      flagErroConnect   = false;  
      timeToErrorConnect  = millis();
      
    } // end else 

    flagVerifierJson  = 0x00;

  } // end if

  flagCalculateInference  = true;
  flagSendJson      = true;

  setupTimer();

} // end verifierJsonConnect

//*****************************************************************************************************************
// Send json connect
//*****************************************************************************************************************

void sendJsonConnect()
{
    
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW);     
    
  StaticJsonDocument<MAX_SIZE_JSON> responseConnectDoc;
  
    char responseMsg[MAX_SIZE_JSON];
  
    responseConnectDoc[headerGSD20]     = status02GSD20;
  readStringLikeChar(SNvalue);
  responseConnectDoc[serialNumberGSD20]   = SNvalue;
    responseConnectDoc[modeOperationGSD20]  = EEPROM.read(modeOperation);
    responseConnectDoc[inferenceGSD20 ]   = EEPROM.read(inferenceValues);
    responseConnectDoc[timeRelay01GSD20]  = EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1);
    responseConnectDoc[modeRelay02GDS20]  = EEPROM.read(relay02Mode);
    responseConnectDoc[timeRelay02GSD20]  = EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1);
    responseConnectDoc[modeLEDGDS20]    = EEPROM.read(ledOperation);
    responseConnectDoc[modeGainGDS20]     = EEPROM.read(gainAmpOp); 
  
    serializeJson(responseConnectDoc, responseMsg); 

  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    //Serial2.write(responseMsg);
    Serial1.write(responseMsg);
    //Serial2.write("\n");    
    Serial1.write("\n");    
  
  } // end if 
    
  else
  {

    Serial.write(responseMsg);
    Serial.write("\n");

  } // end else   
  
} // end sendJsonConnect

//*****************************************************************************************************************
// Send json fail connect
//*****************************************************************************************************************

void sendJsonFailConnect()
{
  
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW); 
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW);   
  
  StaticJsonDocument<MAX_SIZE_JSON> responseConnectFailDoc;
  
  char responseMsg[MAX_SIZE_JSON];
  
  responseConnectFailDoc[headerGSD20]     = status03GSD20;
  readStringLikeChar(SNvalue);
  responseConnectFailDoc[serialNumberGSD20]   = SNvalue;
  
  serializeJson(responseConnectFailDoc, responseMsg);
  
  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    //Serial2.write(responseMsg);
    Serial1.write(responseMsg);
    //Serial2.write("\n");    
    Serial1.write("\n");    
  
  } // end if 
  
  else
  {
  
    Serial.write(responseMsg);
    Serial.write("\n");
    
  } // end else
  
  flagCalculateInference  = true;
  flagSendJson      = true; 
  
} // end sendJsonFailConnect

//*****************************************************************************************************************
// Send Json config
//*****************************************************************************************************************

void sendJsonConfig()
{
  
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW); 
  
  StaticJsonDocument<MAX_SIZE_JSON> responseConfigDoc;
  
    char responseMsg[MAX_SIZE_JSON];
  
    responseConfigDoc[headerGSD20]      = status06GSD20;
  readStringLikeChar(SNvalue);
  responseConfigDoc[serialNumberGSD20]  = SNvalue;
    responseConfigDoc[modeOperationGSD20]   = EEPROM.read(modeOperation);
    responseConfigDoc[inferenceGSD20  ]   = EEPROM.read(inferenceValues);
    responseConfigDoc[timeRelay01GSD20]   = EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1);
    responseConfigDoc[modeRelay02GDS20]   = EEPROM.read(relay02Mode);
    responseConfigDoc[timeRelay02GSD20]   = EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1);
    responseConfigDoc[modeLEDGDS20]     = EEPROM.read(ledOperation);
    responseConfigDoc[modeGainGDS20]    = EEPROM.read(gainAmpOp);
  
    serializeJson(responseConfigDoc, responseMsg);
  
  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    //Serial2.write(responseMsg);
    Serial1.write(responseMsg);
    //Serial2.write("\n");    
    Serial1.write("\n");    
  
  } // end if   
  
  else
  {
    
    Serial.write(responseMsg);
    Serial.write("\n");
    
  } // end else

} // end sendJsonConfig

//*****************************************************************************************************************
// Send json fail config
//*****************************************************************************************************************

void sendJsonFailConfig()
{
  
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW); 
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW); 
  
  StaticJsonDocument<MAX_SIZE_JSON> responseConfigFailDoc;
  
  char responseMsg[MAX_SIZE_JSON];
  
  responseConfigFailDoc[headerGSD20]      = status07GSD20;
  readStringLikeChar(SNvalue);
  responseConfigFailDoc[serialNumberGSD20]  = SNvalue;
  
  serializeJson(responseConfigFailDoc, responseMsg);
  
  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    Serial1.write(responseMsg);
    //Serial2.write(responseMsg);
    Serial1.write("\n");    
    //Serial2.write("\n");    
  
  } // end if   
  
  else
  {
    
    Serial.write(responseMsg);
    Serial.write("\n");
    
  } // end else
        
} // end sendJsonFailConfig

//*****************************************************************************************************************
// Verifier keepalive
//*****************************************************************************************************************

void verifierKeepAlive()
{
  
  while(keepAliveCounter < maxKeepAlive)
  {
    
    keepAliveCounter++;
    sendKeepAlive();
    keepAliveTime = millis();
    
    while(millis() - keepAliveTime < keepAliveTimeToWait)
    {
      
      if(EEPROM.read(modeOperation) == 0x01)
      {
    
        //if (Serial2.available() > 0)
        if (Serial1.available() > 0)
        {
                    
          StaticJsonDocument<MAX_SIZE_JSON> keepAliveDoc;
          
          DeserializationError error = deserializeJson(keepAliveDoc, Serial1);
          
          // Verifier HEADER
          if    (keepAliveDoc.containsKey(headerGSD20))
          {
          
            // Verifier keepAlive
            if    (strcmp(keepAliveDoc[headerGSD20],  status04GSD20) == 0)
            {
              
              sendKeepAlive();
              
              keepAliveCounter = 0;
            
            } // end if
            
            else if (strcmp(keepAliveDoc[headerGSD20],  status05GSD20) == 0)
            {
              
              if(keepAliveDoc.containsKey ("MODE")  &&
                keepAliveDoc.containsKey("INFE")  &&
                keepAliveDoc.containsKey("TRL1")  &&
                keepAliveDoc.containsKey("MRL2")  &&
                keepAliveDoc.containsKey("TRL2")  &&
                keepAliveDoc.containsKey("MLED")  &&
                keepAliveDoc.containsKey("GAIN"))
              {
                                                
                saveValueEEPROM(keepAliveDoc["MODE"], keepAliveDoc["INFE"], keepAliveDoc["TRL1"], keepAliveDoc["MRL2"], keepAliveDoc["TRL2"], keepAliveDoc["MLED"], keepAliveDoc["GAIN"]);
                
                if(flagErrorData) { sendJsonConfig(); } // end if                 
                              
              } // end if 
              
              //else  { sendJsonFailConfig(); }
                
              
            } // end else if  
  
            else if (strcmp(keepAliveDoc[headerGSD20],  status08GSD20) == 0)
            {
              
              if(keepAliveDoc.containsKey ("MRL1"))
              {
                
                EEPROM.write(relay01Mode,         keepAliveDoc["MRL1"]);
                EEPROM.commit();
                
                sendResetSucess();
                
                writeStringToEEPROM(serialNumberFinalGSD20, serialNumberTestGSD20);
                
              } // end if
              
            } // end else if
  
            //else    { sendJsonFailConfig(); }
  
          } // end if

          //else      { sendJsonFailConfig(); }
            
        } // end if
      
      } // end if
      
      else
      {
          
        if (Serial.available() > 0)
        { 
          
          StaticJsonDocument<MAX_SIZE_JSON> keepAliveDoc;
          
          DeserializationError error = deserializeJson(keepAliveDoc, Serial);
          
          // Verifier HEADER
          if(keepAliveDoc.containsKey(headerGSD20))
          {
          
            // Verifier keepAlive
            if (strcmp(keepAliveDoc[headerGSD20],   status04GSD20) == 0)
            {
              
              sendKeepAlive();
              
              keepAliveCounter = 0;
            
            } // end if
            
            else if (strcmp(keepAliveDoc[headerGSD20],  status05GSD20) == 0)
            {
              
              if(keepAliveDoc.containsKey ("MODE")  &&
                keepAliveDoc.containsKey("INFE")  &&
                keepAliveDoc.containsKey("TRL1")  &&
                keepAliveDoc.containsKey("MRL2")  &&
                keepAliveDoc.containsKey("TRL2")  &&
                keepAliveDoc.containsKey("MLED")  &&
                keepAliveDoc.containsKey("GAIN"))
              {
                                                
                saveValueEEPROM(keepAliveDoc["MODE"], keepAliveDoc["INFE"], keepAliveDoc["TRL1"], keepAliveDoc["MRL2"], keepAliveDoc["TRL2"], keepAliveDoc["MLED"], keepAliveDoc["GAIN"]);
                
                if(flagErrorData) { sendJsonConfig(); } // end if 
                              
              } // end if 
              
              //else  { sendJsonFailConfig(); }
                
              
            } // end else if  
  
            else if (strcmp(keepAliveDoc[headerGSD20],  status08GSD20) == 0)
            {
              
              if(keepAliveDoc.containsKey ("MRL1"))
              {
                
                EEPROM.write(relay01Mode,         keepAliveDoc["MRL1"]);
                EEPROM.commit();
                
                sendResetSucess();
                
                writeStringToEEPROM(serialNumberFinalGSD20, serialNumberTestGSD20);
                
              } // end if
              
            } // end else if
  
            //else    { sendJsonFailConfig(); }
  
          } // end if
          
          //else      { sendJsonFailConfig(); }
        
        } // end if
        
      } // end else
      
    } // end while
    
  } // end while
  
  flagCalculateInference  = true;
  flagSendJson      = true;
  
  setupTimer();

  keepAliveCounter    = 0;  
  
} // end verifierKeepAlive

//*****************************************************************************************************************
// Send Keep Alive
//*****************************************************************************************************************

void sendKeepAlive()
{
  
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/5)*timeToWait);
  digitalWrite(pinToLED,  LOW);
  
  StaticJsonDocument<MAX_SIZE_JSON> responseKeepAlive;
  
  char responseMsg[MAX_SIZE_JSON];
  
  responseKeepAlive[headerGSD20]    = status04GSD20;
  
  serializeJson(responseKeepAlive, responseMsg);

  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    //Serial2.write(responseMsg);
    Serial1.write(responseMsg);
    //Serial2.write("\n");    
    Serial1.write("\n");    
  
  } // end if   
  
  else
  {

    Serial.write(responseMsg);
    Serial.write("\n"); 

  } // end else   

} // end sendKeepAlive

//*****************************************************************************************************************
// Send reset sucess
//*****************************************************************************************************************

void sendResetSucess()
{
  
  digitalWrite(pinToLED,  HIGH);
  delay((timeDefaultRelays/2)*timeToWait);
  digitalWrite(pinToLED,  LOW); 
  
  StaticJsonDocument<MAX_SIZE_JSON> responseConfigDoc;
  
    char responseMsg[MAX_SIZE_JSON];
  
    responseConfigDoc[headerGSD20]      = status09GSD20;
  readStringLikeChar(SNvalue);
  responseConfigDoc[serialNumberGSD20]  = SNvalue;
    responseConfigDoc[modeOperationGSD20]   = EEPROM.read(modeOperation);
    responseConfigDoc[inferenceGSD20  ]   = EEPROM.read(inferenceValues);
    responseConfigDoc[modeRelay01GDS20  ]   = EEPROM.read(relay01Mode);
    responseConfigDoc[timeRelay01GSD20]   = EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1);
    responseConfigDoc[modeRelay02GDS20]   = EEPROM.read(relay02Mode);
    responseConfigDoc[timeRelay02GSD20]   = EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1);
    responseConfigDoc[modeLEDGDS20]     = EEPROM.read(ledOperation);
    responseConfigDoc[modeGainGDS20]    = EEPROM.read(gainAmpOp);
  
    serializeJson(responseConfigDoc, responseMsg);
  
  if(EEPROM.read(modeOperation) == 0x01)
  {
  
    //Serial2.write(responseMsg);
    Serial1.write(responseMsg);
    //Serial2.write("\n");    
    Serial1.write("\n");    
  
  } // end if   
  
  else
  {
    
    Serial.write(responseMsg);
    Serial.write("\n");
    
  } // end else

} // end sendResetSucess



//*****************************************************************************************************************
// Write string to EEPROM
//*****************************************************************************************************************

void writeStringToEEPROM(char add, String data)
{
  
  int sizeString = data.length();

  for(int i = 0; i < sizeString; i++) EEPROM.write(add + i, data[i]);
  
  EEPROM.commit();
  
} // end writeStringToEEPROM

//*****************************************************************************************************************
// Read string like char
//*****************************************************************************************************************

void readStringLikeChar(char data[])
{
  
  char value[MAX_SIZE_SN];
  
  for (int i = 0; i < MAX_SIZE_SN; i++) value[i] = EEPROM.read(i + serialNumberFinalGSD20);
  
  strncpy(data, value, MAX_SIZE_SN);

} // end readStringLikeChar

//*****************************************************************************************************************
// Save value EEPROM
//*****************************************************************************************************************

void saveValueEEPROM(int mode, int infe, int trl1, int mrl2, int trl2, int mled, int gain)
{
  
  /*
  if    (infe < 1)  infe = 1;
  else if (infe > 8)  infe = 8;
  
  if    (trl1 < 100)  trl1 = 100;
  else if (trl1 < 5000) trl1 = 5000;
  
  if    (mrl2 > 2)  mrl2 = 2;

  if    (trl2 < 100)  trl2 = 100;
  else if (trl2 < 5000) trl2 = 5000;
  
  if    (mled > 1)  mled = 1;
  
  if    (gain < 1)  gain = 1;
  else if (gain > 100)  gain = 100;
  */  
  
  // To verifier
  if( (mode < 0)    ||  (mode > 1)    ||
    (infe < 1)    ||  (infe > 100)  ||
    (trl1 < 100)  ||  (trl1 > 5000) ||
    (mrl2 < 0)    ||  (mrl2 > 2)    ||
    (trl2 < 100)  ||  (trl2 > 5000) ||
    (mled < 0)    ||  (mled > 2)    ||
    (gain < 1)    ||  (gain > 100))
  {     flagErrorData = false;  } // end if
  
  else  { flagErrorData = true;   } // end else 
  
  // To save
  if(flagErrorData)
  {
    
    EEPROM.write(modeOperation,       mode);
    EEPROM.write(inferenceValues,       infe);
    EEPROM.write(timeRelay01Mode,       trl1/256);
    EEPROM.write(timeRelay01Mode+1,     trl1%256);
    EEPROM.write(relay02Mode,         mrl2);
    EEPROM.write(timeRelay02Mode,       trl2/256);
    EEPROM.write(timeRelay02Mode+1,     trl2%256);
    EEPROM.write(ledOperation,        mled);
    EEPROM.write(gainAmpOp,         gain);  
    
    EEPROM.commit();  
    
    valueInferenceNow = EEPROM.read(inferenceValues);
    valueGain     = EEPROM.read(gainAmpOp);
    timeToRelayFix01  =   (EEPROM.read(timeRelay01Mode)*256 + EEPROM.read(timeRelay01Mode+1));
    timeToRelayFix02  =   (EEPROM.read(timeRelay02Mode)*256 + EEPROM.read(timeRelay02Mode+1));  
    
    valueGain     = (gain*FIX_NORMALIZE_VARIABLE);
    valueInferenceNow =   ((infe*FIX_MAX_INFE)/(MAX_RATE));
    
    if(valueGain      < MIN_VALUE_MCP41010) valueGain     = MIN_VALUE_MCP41010;
    if(valueInferenceNow  < MIN_VALUE_INFE)   valueInferenceNow = MIN_VALUE_INFE;
    
    digitalPotWrite(valueGain);
  
  } // end if
  
  // else  { sendJsonFailConfig();   } // end
  
} // end saveValueEEPROM

//*****************************************************************************************************************
// Interface LED
//*****************************************************************************************************************

void interfaceLED()
{
  
  if(flagTurnOnLED  == 0x01)  digitalWrite(pinToLED,  HIGH);
  else              digitalWrite(pinToLED,  LOW);
  
} // end interfaceLED

//*****************************************************************************************************************
// Interface Relay
//*****************************************************************************************************************

void interfaceRELAY()
{
  
  if((EEPROM.read(relay01Mode)  == 0x01))
  {
    
    if    (flagTurnRelay01  ==  0x01) 
    {
      
        digitalWrite(pinToRelay01,  LOW);
      
    } // end if
    else  digitalWrite(pinToRelay01,  HIGH);
    
  } // end if

  
  if    (flagTurnRelay02  ==  0x01) digitalWrite(pinToRelay02,  LOW);
  else if (flagTurnRelay02  ==  0x02) digitalWrite(pinToRelay02,  HIGH);
  else                  digitalWrite(pinToRelay02,  HIGH);
  
} // end interfaceRELAY

//*****************************************************************************************************************
// To write MCP41010
//*****************************************************************************************************************

void digitalPotWrite(int value)
{
  
  digitalWrite(pinCS1,  LOW);
  digitalWrite(pinCS2,  LOW);
  digitalWrite(pinCS3,  LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(pinCS1,  HIGH);
  digitalWrite(pinCS2,  HIGH);
  digitalWrite(pinCS3,  HIGH);

} // end digitalPotWrite
