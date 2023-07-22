#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <numeric>
#include "arduinoFFT.h"
#include "arduinoFFT.cpp"

#include "mfcclib.h"
#include "fbank_data.h"




// variáveis globais
#ifdef computer
	const double PI = 4*atan(1.0);   // Pi = 3.14...
#endif
// int freqsamp;
// size_t winLengthSamples, frameShiftSamples, numCepstra, numFFT, numFFTBins, numFilters;
// double preEmphCoef, lowFreq, highFreq;
// v_d frame, powerSpectralCoef, lmfbCoef, hamming, mfcc, prevsamples;
// m_d fbank, dct;

double *samples_imag; //samples imaginario com zeros
double *samples_real; //samples imaginario com zeros
v_d powerSpectralCoef;
v_d prevsamples;
int numFFTBins;

arduinoFFT FFT = arduinoFFT(); 
v_d hamming; 
m_d dct;
// m_d fbank;
twmap twiddle;



//// Funções auxiliares ////
// Hertz to Mel conversion
double hz2mel (double f) {
        return 2595*std::log10(1+f/700);
    }
     
// Mel to Hertz conversion
double mel2hz (double m) {
        return 700*(pow(10,m/2595)-1);
    }
       
// Twiddle factor computation
void compTwiddle(twmap &twiddle, int numFFT) {
        const c_d J(0,1);      // Imaginary number 'j'

        for (int N=2; N<=numFFT; N*=2)
            for (int k=0; k<=N/2-1; k++)
			{
                twiddle[N][k] = exp(-2*PI*k/N*J);
				// Serial.printf("N[%d] K[%d] Free Heap: %d\n", N, k, ESP.getFreeHeap());
			}
}
    

// Cooley-Tukey DIT-FFT recursive function
v_c_d fft(v_c_d x, twmap &twiddle) {
	int N = x.size();
    if (N==1)
		return x;
    
    v_c_d xe(N/2,0), xo(N/2,0), Xjo, Xjo2;
    int i;
    
    // Construct arrays from even and odd indices
    for (i=0; i<N; i+=2)
		xe[i/2] = x[i];
    for (i=1; i<N; i+=2)
		xo[(i-1)/2] = x[i];
    
	// Compute N/2-point FFT
	Xjo = fft(xe, twiddle);
	Xjo2 = fft(xo, twiddle);
	Xjo.insert (Xjo.end(), Xjo2.begin(), Xjo2.end());
    
	// Butterfly computations
	for (i=0; i<=N/2-1; i++) {
		c_d t = Xjo[i], tw = twiddle[N][i];
		Xjo[i] = t + tw * Xjo[i+N/2];
		Xjo[i+N/2] = t - tw * Xjo[i+N/2];
	}
	return Xjo;
}

//// Calculo da MFCC de um quadro (HAMMING // PSD // LMFB // DCT  /////
// O cálculo da MFCC tem as seguintes etapas: pré ênfase da jalena da hamming,
// cálculo da densidade espectral de potência, a separação em frequências na escala MEL 
// e o banco de filtros para cada faixa, e a transformada discreta do cosseno da saída do banco de filtros

// Pre-emphasis and Hamming window
void preEmphHam(v_d &frame, double preEmphCoef, v_d &hamming) {
	v_d procFrame(frame.size(), hamming[0]*frame[0]);
	for (int i=1; i<frame.size(); i++)
		procFrame[i] = hamming[i] * (frame[i] - preEmphCoef * frame[i-1]);
		frame = procFrame;
}

// Power spectrum computation
void computePowerSpec(v_d &frame, int numFFT) {
	
	frame.resize(numFFT); // Pads zeros
	//v_c_d framec (frame.begin(), frame.end()); // Complex frame
	//v_c_d fftc = fft(framec);
    
    std::copy(frame.begin(), frame.end(), samples_real);
    for (int index=0; index<numFFT; index++)
	{
		samples_imag[index]=0;
    }
     
	//fft	
    FFT.Windowing(samples_real, numFFT, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
    FFT.Compute(samples_real, samples_imag, numFFT, FFT_FORWARD); // Compute FFT 
    // FFT.ComplexToMagnitude(samples_real, samples_imag, numFFT); // Compute magnitudes 
    

	for (int i=0; i<numFFTBins; i++){
		//powerSpectralCoef[i] = samples_real[i]/3;
		powerSpectralCoef[i] = pow(samples_real[i],2)+pow(samples_imag[i],2);
		// printf("powerSpectralCoef[%d] = %f\n", i, powerSpectralCoef[i]);
	}

}	

// Applying log Mel filterbank (LMFB)
void applyLMFB(v_d &powerSpectralCoef, const double fbank[16][513], v_d &lmfbCoef, int numFilters) {
	lmfbCoef.assign(numFilters,0);
      
	for (int i=0; i<numFilters; i++) {
		// Multiply the filterbank matrix
		for (int j=0; j< numFFTBins; j++)
			lmfbCoef[i] += fbank[i][j] * powerSpectralCoef[j];
		
		// Apply Mel-flooring
		if (lmfbCoef[i] < 1.0)
			lmfbCoef[i] = 1.0;
	}
        
	// Applying log on amplitude
	for (int i=0; i<numFilters; i++)
	{
		lmfbCoef[i] = std::log (lmfbCoef[i]);
	}

}
    
// Computing discrete cosine transform
void applyDct(m_d &dct, v_d &lmfbCoef, v_d &mfcc, int numFilters, int numCepstra) {
	mfcc.assign(numCepstra+1,0);
	for (int i=0; i<=numCepstra; i++) {
		for (int j=0; j<numFilters; j++)
		{
			mfcc[i] += dct[i][j] * lmfbCoef[j];
		}
	}
}


//// ROTINAS DE INICIALIZAÇÃO ////
// Pre-computing Hamming window and dct matrix
void initHamDct(v_d &hamming, m_d &dct, int numFilters, int numCepstra, int winLengthSamples) {
	int i, j;

	hamming.assign(winLengthSamples, 0);
	for (i=0; i<winLengthSamples; i++)
		hamming[i] = 0.54 - 0.46 * cos(2 * PI * i / (winLengthSamples-1));

	v_d v1(numCepstra+1,0), v2(numFilters,0);
	for (i=0; i <= numCepstra; i++)
		v1[i] = i;
	for (i=0; i < numFilters; i++)
		v2[i] = i + 0.5;


	dct.reserve (numFilters*(numCepstra+1)); 
	double c = sqrt(2.0/numFilters);
	for (i=0; i<=numCepstra; i++) {
		v_d dtemp;
		for (j=0; j<numFilters; j++)
			dtemp.push_back (c * cos(PI / numFilters * v1[i] * v2[j]));
		dct.push_back(dtemp);
	}
}

// Precompute filterbank
void initFilterbank(m_d &fbank, int numFilters, int numFFTBins, int freqsamp, double lowFreq, double highFreq) {
	// Convert low and high frequencies to Mel scale
	double lowFreqMel = hz2mel(lowFreq);
	double highFreqMel = hz2mel (highFreq);

	// Calculate filter centre-frequencies
	v_d filterCentreFreq;
	filterCentreFreq.reserve (numFilters+2);
	for (int i=0; i<numFilters+2; i++)
		filterCentreFreq.push_back (mel2hz(lowFreqMel + (highFreqMel-lowFreqMel)/(numFilters+1)*i));

	// Calculate FFT bin frequencies
	v_d fftBinFreq;
	fftBinFreq.reserve(numFFTBins);
	for (int i=0; i<numFFTBins; i++)
		fftBinFreq.push_back (freqsamp/2.0/(numFFTBins-1)*i);
				
	// Filterbank: Allocate memory
	fbank.reserve (numFilters*numFFTBins);
        
	// Populate the fbank matrix
	for (int filt=1; filt<=numFilters; filt++) {
		v_d ftemp;
		for (int bin=0; bin<numFFTBins; bin++) {
			double weight;
			if (fftBinFreq[bin] < filterCentreFreq[filt-1])
				weight = 0;
			else if (fftBinFreq[bin] <= filterCentreFreq[filt])
				weight = (fftBinFreq[bin] - filterCentreFreq[filt-1]) / (filterCentreFreq[filt] - filterCentreFreq[filt-1]);
			else if (fftBinFreq[bin] <= filterCentreFreq[filt+1])
				weight = (filterCentreFreq[filt+1] - fftBinFreq[bin]) / (filterCentreFreq[filt+1] - filterCentreFreq[filt]);
			else
				weight = 0;
			ftemp.push_back (weight);
		}
		fbank.push_back(ftemp);
	}
}


// Cálculo das variáveis globais da MFCC, inicialização da MFCC
void  MFCC_INIT(int fft_size, int sampFreq, int nCep, 
				int frameShift, int numFilt, double lf, double hf) {

		#ifdef computer
		// printf("Freq Sample : %d\n", sampFreq);
		// printf("Num Cepstra : %d\n", nCep);
		// printf("Num Filters : %d\n", numFilt);
		// printf("FFT   Size  : %d\n", fft_size);
		// printf("Frame Shift : %d\n", frameShift);
		#else
		Serial.printf("Freq Sample : %d\n", sampFreq);
		Serial.printf("Num Cepstra : %d\n", nCep);
		Serial.printf("Num Filters : %d\n", numFilt);
		Serial.printf("FFT   Size  : %d\n", fft_size);
		Serial.printf("Frame Shift : %d\n", frameShift);
		#endif

        numFFTBins = fft_size/2 + 1;
		powerSpectralCoef.assign (numFFTBins, 0);
		prevsamples.assign (fft_size - frameShift, 0);

		samples_imag = (double*) malloc(fft_size * sizeof(double));
		samples_real = (double*) malloc(fft_size * sizeof(double));


		#ifdef computer
		// printf("Init Filterbank\n");
		#else
		Serial.printf("Init Filterbank\n");
		#endif
        // initFilterbank(fbank, numFilt, numFFTBins, sampFreq, lf, hf);
		#ifdef computer
		// printf("Init Hamming and DCT\n");
		#else
		Serial.printf("Init Hamming and DCT\n");
		#endif
        initHamDct(hamming, dct, numFilt, nCep, fft_size);
		#ifdef computer
		// printf("Init Twiddle\n");
		#else
		Serial.printf("Init Twiddle\n");
		#endif
        compTwiddle(twiddle, fft_size);
        
		#ifdef computer
		// printf("MFCC_INIT done\n");
		#else
		Serial.printf("MFCC_INIT done\n");
		Serial.printf("Free Heap: %d\n", ESP.getFreeHeap());
		#endif
    }

// Process each frame and extract MFCC
v_d mfcc_processFrame(int16_t *samples, int N, size_t frameShift, int fft_size, int numFilters, int numCepstra) { // size_t N) {
	

	v_d frame ;
	v_d mfcc;
	v_d lmfbCoef;

	frame = prevsamples;
	for (int i=0; i<N; i++)
		frame.push_back(samples[i]);
 

	prevsamples.assign(frame.begin()+frameShift, frame.end());

	computePowerSpec(frame, fft_size);
	applyLMFB(powerSpectralCoef, fbank, lmfbCoef, numFilters);
	applyDct(dct, lmfbCoef, mfcc, numFilters, numCepstra);
	
	return mfcc; 
}


