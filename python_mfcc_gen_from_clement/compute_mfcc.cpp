#include "mfcclib.h"
#include "mfcclib.cpp"

#define __lowFreq      50
#define __highFreq     __samplingRate/2
#define __numFilters   16
#define __numCepstra   15
#define __samplingRate 8000
#ifdef FFT1024
  #define __FFT_SIZE     1024
  #define __winLength    128 // (__FFT_SIZE ) / (__samplingRate / 1000)   // In milliseconds
  #define __frameShift   58  // (1000 - __winLength) / (__numFilters - 1) // In milliseconds
  #define __mfcc_step    464 // (__frameShift) * (__samplingRate / 1000) // In samples
#elif FFT512
  #define __FFT_SIZE     512
  #define __winLength    64  // (__FFT_SIZE ) / (__samplingRate / 1000)   // In milliseconds
  #define __frameShift   62  // (1000 - __winLength) / (__numFilters - 1) // In milliseconds
  #define __mfcc_step    496  // (__frameShift) * (__samplingRate / 1000) // In samples
#else
  #define __FFT_SIZE     2048
  #define __winLength    256 // (__FFT_SIZE ) / (__samplingRate / 1000)   // In milliseconds  
  #define __frameShift   46  // (1000 - __winLength) / (__numFilters - 1) // In milliseconds
  #define __mfcc_step    368 // (__frameShift) * (__samplingRate / 1000) // In samples
#endif 
int16_t vReal[__samplingRate] = {}; 
int16_t vReal_janela[__mfcc_step]; 
v_d mfcc_output, mfcc_output_frame;

int main()
{
    MFCC_INIT(__FFT_SIZE, __samplingRate, __numCepstra, __winLength, __frameShift, __numFilters, __lowFreq, __highFreq);

    int input_buffer_index = 0;

    FILE* series = fopen("time_series_1s.dat", "rb");

    while(fscanf(series, "%hd", &vReal[input_buffer_index]) != EOF)
      input_buffer_index++;
    fclose(series);

    for(int mfcc_index = 0; mfcc_index < 8000; mfcc_index+=__mfcc_step)
    {
      for(int array_copy_index = 0; array_copy_index< __mfcc_step; array_copy_index++)
      {
        vReal_janela[array_copy_index]=vReal[mfcc_index+array_copy_index];
        // printf("vReal_janela[%d] = %d\n", array_copy_index, vReal_janela[array_copy_index]);
      }

      mfcc_output_frame = mfcc_processFrame(vReal_janela, __mfcc_step, __frameShift, __FFT_SIZE, __numFilters, __numCepstra);
      mfcc_output.insert(mfcc_output.end(), mfcc_output_frame.begin(), mfcc_output_frame.end());
    }

    FILE *mfcc_output_file = fopen("mfcc_from_time_series_1s.dat","w+");
	
	for(int output_buffer_index = 64; output_buffer_index < 320; output_buffer_index++) //256
	{
		fprintf(mfcc_output_file, "%f\n", mfcc_output[output_buffer_index]);
    // printf("mfcc[%d] = %f\n", output_buffer_index, mfcc_output[output_buffer_index]);
	}

	fclose(mfcc_output_file);
}