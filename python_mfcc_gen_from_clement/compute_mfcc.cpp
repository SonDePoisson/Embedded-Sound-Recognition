#include "mfcclib.h"
#include "mfcclib.cpp"

#define __lowFreq      50
#define __highFreq     __samplingRate/2
#define __numFilters   16   // Freq
#define __numCepstra   15   // Temps (0 -> 15 = 16 frames)
#define __samplingRate 8000
#define __FFT_SIZE     1024  // Sample
#define __frameShift   465   // Sample// (__frameShift) * (__samplingRate / 1000)  // In samples
int16_t vReal[__samplingRate] = {}; 
int16_t vReal_janela[__frameShift]; 
v_d mfcc_output, mfcc_output_frame;

int main()
{
  MFCC_INIT(__FFT_SIZE, __samplingRate, __numCepstra, __frameShift, __numFilters, __lowFreq, __highFreq);

  int input_buffer_index = 0;

  FILE* series = fopen("time_series_1s.dat", "rb");

  while(fscanf(series, "%hd", &vReal[input_buffer_index]) != EOF)
    input_buffer_index++;
  fclose(series);

  // Nexts windows with frameshift
  for(int mfcc_index = 0; mfcc_index < 8000; mfcc_index+=__frameShift)
  {
    for(int array_copy_index = 0; array_copy_index< __frameShift; array_copy_index++)
      vReal_janela[array_copy_index]=vReal[mfcc_index+array_copy_index];

    mfcc_output_frame = mfcc_processFrame(vReal_janela, __frameShift, __frameShift, __FFT_SIZE, __numFilters, __numCepstra);
    mfcc_output.insert(mfcc_output.end(), mfcc_output_frame.begin(), mfcc_output_frame.end());
  }

  FILE *mfcc_output_file = fopen("mfcc_from_time_series_1s.dat","w+");
	
	for(int output_buffer_index = 0; output_buffer_index < 256; output_buffer_index++) //256
	{
		fprintf(mfcc_output_file, "%f\n", mfcc_output[output_buffer_index]);
    // printf("mfcc[%d] = %f\n", output_buffer_index, mfcc_output[output_buffer_index]);
	}

	fclose(mfcc_output_file);
}