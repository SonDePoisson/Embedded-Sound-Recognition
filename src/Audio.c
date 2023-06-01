#include <Arduino.h>
#include "Audio.h"

// Taille de l'en-tête wav
#define WAV_HEADER_SIZE 0x2c

// Position et taille du champs FILESIZE dans l'en-tête
#define WAV_FILESIZE_OFFSET 0x04
#define WAV_FILESIZE_LENGTH 0x04

void exit_if(int cond, const char *prefix)
{
  if (!cond) return;
  perror(prefix);
  exit(EXIT_FAILURE);
}

// Fonctions utilisé pour la lecture du signal
/**
 * Extract `size` bytes as little endian value from `data` (starting at `offset`)
 * Return the native representation.
 */
long long extract_le2n(int size, unsigned char data[], int offset)
{
  unsigned long long res = 0;
  while (size --)
    res = res << 8 | data[offset + size];
  return res;
}

/**
 * Lit l'en-tête d'un fichier `WAV` depuis le descripteur de ficher `fd` et
 * stocke le resultat dans `header`.
 * Retourne -1 en cas d'erreur.
 *
 * RQ: La taille de l'en-tête est WAV_HEADER_SIZE
 */
int wavefile_read_header(FILE *fd, unsigned char header[])
{
    if(fread(header, WAV_HEADER_SIZE, 1, fd) < 1)
    {
        return -1;
    }
    return 0;
}

/**
 * Extrait la longueur d'un fichier WAV depuis un en-tête (en little endian).
 * La taille est représentée sur 32 bits (long). La position dans l'en-tête est
 * WAV_FILESIZE_OFFSET.
 *
 * RQ: La taille contenue dans l'en-tête est relative à la position (offset) du
 * champ taille (il faudra donc le corriger de WAV_HEADER_SIZE -
 * WAV_FILESIZE_OFFSET - WAV_FILESIZE_LENGTH)
 */
unsigned long wavefile_length(unsigned char header[])
{
    return extract_le2n(WAV_FILESIZE_LENGTH, header, WAV_FILESIZE_OFFSET) 
              - WAV_HEADER_SIZE + WAV_FILESIZE_OFFSET + WAV_FILESIZE_OFFSET;
}

/**
 * Lit le contenu du signal depuis `fd` et stocke le resultat dans `signal`.
 * La zone mémoire pointée par signal est supposée assez grande pour stocker
 * l'intégralité du signal.  Retourne -1 en cas d'erreur.
 *
 * RQ: Comment s'assurer de ne pas faire de buffer-overflow ? (si vous voyez
 * comment, le faire)
 */
int wavefile_read_signal(FILE *fd, double *signal)
{
    // TODO but after `wavefile_read`
    return -1;
}

////////// Lecture/Écriture de fichier WAV ///////////
/**
 * Lit un fichier WAV depuis le descripteur `fd`. L'en-tête du fichier sera
 * stocké dans `header` et la taille directement accessible depuis
 * `signal_size` (s'il est non NULL).
 *
 * Cette fonction retourne le fichier, qui sera stocké dans tableau allouée par
 * malloc (pensez à libérer). En cas d'erreur elle retournera NULL
 */
double *wavefile_read(FILE *fd, unsigned char header[], long *signal_size)
{
  // TODO
  // - lire l'en-tête
  wavefile_read_header(fd, header);
  // - Extraire la taille
  long size = wavefile_length(header);
  // fprintf(stderr, "Read file of length %lu\n", size);
  // - Allouer un tableau de taille*double
  double *signal = malloc(size*sizeof(double));
  // - Lire le signal
  for (unsigned long i = 0; i < size; i++)
  {
    signal[i] = fgetc(fd);
  }
  // - Retourner ce qui doit l'être
  *signal_size = size;
  return signal;
}
/**
 * Cette fonction écrit le signal `data` de taille `signal_size`, dans un
 * fichier wav l'en-tête doit être valide et compatible.
 */
void wavefile_write(FILE *fd, unsigned char header[], long signal_size, const double *data)
{
  fwrite(header, WAV_HEADER_SIZE, 1, fd);
  while(signal_size --)
    putc((char)round(*(data++)), fd);
}

////////// Traitement du signal ///////////

/**
 * Fabrique un nouveau filtre `gaussien` de caracteristique sigma.
 * Retourne un nouveau tableau contenant le filtre de taille `taille`.
 */
double *gaussienne(double sigma, int *taille)
{
  double *gauss;
  double s = 0.0;

  int m = 2 * (int) ceil(3.0 * sigma) + 1; // largeur de l'intervalle

  if (taille)
    *taille = m; // Stocke la taille

  gauss = malloc(m * sizeof(double));

  for (int i = 0; i < m; i++) {
    double x = (double) (i - m / 2) / sigma; // centrage sur m/2
    gauss[i] = exp(-(x*x) / 2.0);
    s += gauss[i];
  }

  for (int i = 0; i < m; i++)
    gauss[i] /= s;  // normalisation
  return gauss;
}

/**
 * Etant donné un signal `in` de taille `size`, rempli
 * le signal out en `moyenneant` trois points
 *
 * RQ: Attentions aux extrémitées
 */
void mean3(int size, const double *in, double* out)
{
  out[0] = in[0];
  out[size] = in[size];

  // TODO
  for (unsigned long i = 1; i < size-1; i++)
  {
    out[i] = (in[i-1] + in[i] + in[i+1])/3;
  }
}

/**
 * Calcule la convolution `filter` de taille `filter_size` au données `data` de
 * taille compatible.
 *
 * Cette fonction retourne la valeur calculée
 */
double convolute(int filter_size, const double *data, const double *filter)
{
  unsigned long sum = 0;

  for (int i = 0; i < filter_size ; i++)
  {
    sum += (data[i]*filter[filter_size -1 -i]);
    // fprintf(stderr, "Sum: %lu\n", sum);
  }

  return sum;
}

/**
 *
 * Applique un filtre `filter` de taille `filter_size` sur un signal `in` de
 * taille `size`. Le résultat est stocké dans `out`.
 *
 * RQ: - Attentions aux extrémitées
 *     - Tous les filtres ne sont pas de taille 3 !
 */
void filter_signal(int size, const double *in, double *out, int filter_size, const double *filter)
{
  for (int i = 0; i < filter_size/2; i++)
  {
    out[i] = in[i];
    out[size - i] = in[size - i];
  }

  for (unsigned long i = filter_size/2; i < size - (filter_size/2); i++)
  {
    out[i] = convolute(filter_size, &in[i], filter);
  }
}

/**
 * Un exemple de filtre moyenneur
 */
void mean3_bis(int size, const double *in, double* out)
{
  double filter[3] = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
  filter_signal(size, in, out, 3, filter);
}