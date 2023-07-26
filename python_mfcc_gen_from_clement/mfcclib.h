#include <complex>
#include <vector>
#include <map>



// definindo os tipos
typedef std::vector<double> v_d;
typedef std::complex<double> c_d;
typedef std::vector<v_d> m_d;
typedef std::vector<c_d> v_c_d;
typedef std::map<int,std::map<int,c_d> > twmap;


// declarando todas as funções
double hz2mel (double f);
double mel2hz (double m);
void compTwiddle(twmap &twiddle, int numFFT);
v_c_d fft(v_c_d x, twmap &twiddle);
void preEmphHam(v_d &frame, double preEmphCoef, v_d &hamming);
//void computePowerSpec(double *samples); 
void computePowerSpec(v_d &frame, int numFFT); 
void applyLMFB(v_d &powerSpectralCoef, const double **fbank, v_d &lmfbCoef, int numFilters);	
void applyDct(m_d &dct, v_d &lmfbCoef, v_d &mfcc, int numFilters, int numCepstra);
void initHamDct(v_d &hamming, m_d &dct, int numFilters, int numCepstra, int winLengthSamples);
void initFilterbank(m_d &fbank, int numFilters, int numFFTBins, int freqsamp, double lowFreq, double highFreq);
void  MFCC_INIT(int fft_size, int sampFreq, int nCep, 
				int frameShift, int numFilt, double lf, double hf);
v_d mfcc_processFrame(int16_t *samples, int N, int fft_size, int numFilters, int numCepstra);
