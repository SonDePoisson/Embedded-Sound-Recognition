#include <Arduino.h>
#include<iostream>
#include<fstream>
#include<cmath>
#include<vector>
#include <complex> 
#include <bitset> 

#define FS         8             // default sampling rate in KHz, actual value will be obtained from wave file
#define HIGH_FREQ  4             // default high frequency limit in KHz
#define LOW_FREQ   0             // default low frequency limit in KHz
#define FrmLen    25             // frame length in ms
#define FrmSpace  10             // frame space in ms
#define LOGENERGY  1             // whether to include log energy in the output
#define FFTLen   512             // FFT points
#define FiltNum   26             // number of filters
#define PCEP      12             // number of cepstrum

using namespace std;

typedef struct     // WAV stores wave file header
{
        int rId;    
        int rLen;   
        int wId;    
        int fId;    
        int fLen;   
        short wFormatTag;       
        short nChannels;        
        int nSamplesPerSec;   // Sampling rate. This returns to FS variable
        int nAvgBytesPerSec;  // All other parameters are not used in processing
        short nBlockAlign;      
        short wBitsPerSample;   
        int dId;              
        int wSampleLength;    
}WAV;

// function declarations
void InitHamming();
void HammingWindow(short* buf,float* data);
float FrmEnergy(short* data);
void zero_fft(float *buffer,vector<complex<float> >& vec);
void FFT(const unsigned long & fftlen, vector<complex<float> >& vec);
void InitFilt(float (*w)[FFTLen/2+1], int num_filt); 
void CreateFilt(float (*w)[FFTLen/2+1], int num_filt, int Fs, int high, int low);
void mag_square(vector<complex<float> > & vec, vector<float> & vec_mag);
void Mel_EN(float (*w)[FFTLen/2+1],int num_filt, vector<float>& vec_mag, float * M_energy);
void Cepstrum(float *M_energy);