// -----------------------------------------------------------------------------
// Wrapper for MFCC feature extractor
// -----------------------------------------------------------------------------
//
//  Copyright (C) 2016 D S Pavan Kumar
//  dspavankumar [at] gmail [dot] com
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string.h> // For memset
#include "mfcclib.h"
#include "mfcclib.cpp"

//#include <time.h> // só incluir quando for avaliar tempo de processamento no PC


// Main
int main() {
  
    
    // Assign variables
    int numCepstra = 15;
    int numFilters = 16;
    int samplingRate = 8000;
    int winLength = 256;
    int frameShift = 46;
    int lowFreq = 50;
    int highFreq = samplingRate/2;
    int FFT_SIZE=2048;
	unsigned int mfcc_step=368;
  
     // inicializa as variáveis pra MFCC
    MFCC_INIT(samplingRate, numCepstra, winLength, frameShift, numFilters, lowFreq, highFreq);
   
  
    FILE *sampleFile_input;
     
     // Indexs counters
	unsigned int input_buffer_index = 0;
	unsigned int mfcc_index=0;
	unsigned int output_buffer_index=0;
	unsigned int array_copy_index=0;
	unsigned int i=0;
	
	
	// avaliando tempo (comentar tudo quando não estiver avliando tempo)
	//clock_t start, end;
    // double cpu_time_used;
	
	 //tamanho do arquivo sempres 1 segundo, então numero de amostras = taxa de amostragem
	int16_t vReal[samplingRate]; 
	double vReal_janela[FFT_SIZE];
	v_d mfcc, mfcc_frame;
            
     // Initialize the time series
	memset(&vReal, 0, sizeof(vReal)); 
	
	// Open the times series data	
	sampleFile_input = fopen("time_series_1s.dat","rb");
	// Readfile
	
	// Read in the contents of the sample file
	while(fscanf(sampleFile_input, "%hd", &vReal[input_buffer_index]) != EOF) // %lf tells fscanf to read a double
	{
		// printf("%hd\n", vReal[input_buffer_index]);
		input_buffer_index++;
	}
	// Close the sample file
	fclose(sampleFile_input);

      
    //start = clock();
    // calculando as FFTs e MFCCs das janelas
    for(mfcc_index = 0; mfcc_index < 8000; mfcc_index+=mfcc_step)
	{
		
	    for (array_copy_index = 0; array_copy_index< mfcc_step; array_copy_index++)
	    {
			vReal_janela[array_copy_index]=vReal[mfcc_index+array_copy_index];
			// printf("vReal_janela[%d] : %hd\n", array_copy_index, vReal_janela[array_copy_index]);
		}
	                      
		mfcc_frame =  mfcc_processFrame(vReal_janela, mfcc_step);
		mfcc.insert(mfcc.end(), mfcc_frame.begin(), mfcc_frame.end());

		// printf("MFCC[%d] : %f\n", mfcc_index, mfcc[mfcc_index]);

	}
	
	// avaliando tempo de processamento (comentar quando não usando
	//end = clock();
    //cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;	
	
	
	// escrevendo no arquivo de saída
	// Pointer to the sample data file output
	FILE *sampleFile_output;
	
	// Open the output file
	sampleFile_output = fopen("mfcc_from_time_series_1s.dat","w+");
	
	for(output_buffer_index = 64; output_buffer_index < 320; output_buffer_index++) //256
	{
		fprintf(sampleFile_output, "%f\n", mfcc[output_buffer_index]);

		//fprintf(sampleFile_output, "%f\n", cpu_time_used);
		
		// fprintf(sampleFile_output, "%hd\n", vReal_janela[output_buffer_index]);
	}
	//getchar();
	
	// Close the sample file
	fclose(sampleFile_output);


    return 0;
}



