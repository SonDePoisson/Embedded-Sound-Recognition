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
void compTwiddle(void);
v_c_d fft(v_c_d x);
void preEmphHam(void);
//void computePowerSpec(double *samples); 
void computePowerSpec(); 
void applyLMFB(void);	
void applyDct(void);
void initHamDct(void);
void initFilterbank ();
void  MFCC_INIT(int fft_size, int sampFreq, int nCep, int winLength, int frameShift, int numFilt, double lf, double hf);
v_d mfcc_processFrame(double_t *samples, int N);
