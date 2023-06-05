#include <Arduino.h>
#include "Audio.h"

void exit_if(int cond, const char *prefix)
{
  if (!cond) return;
  Serial.println(prefix);
  exit(0);
}

// Fonctions utilisé pour la lecture du signal
/**
 * Extract `size` bytes as little endian value from `data` (starting at `offset`)
 * Return the native representation.
 */
long long extract_le2n(int size, char data[], int offset)
{
  unsigned long long res = 0;
  while (size --)
    res = res << 8 | data[offset + size];
  return res;
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
unsigned long wavefile_length(char header[])
{
    return extract_le2n(WAV_FILESIZE_LENGTH, header, WAV_FILESIZE_OFFSET) 
              - WAV_HEADER_SIZE + WAV_FILESIZE_OFFSET + WAV_FILESIZE_OFFSET;
}

void print_file_parameters(File fd)
{
  Serial.printf("\n");
  Serial.printf("File type: %s\n", fd.isDirectory() ? "Directory" : "File");
  exit_if(fd.isDirectory(), "File is a directory");
  
  Serial.printf("File name: %s\n", fd.name());

  Serial.printf("File size: %d\n", fd.size());
  exit_if(fd.size() == 0, "File Empty");
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
void wavefile_read(char *file, struct signal *signal)
{ 
  exit_if(!SPIFFS.begin(), "An Error has occurred while mounting SPIFFS");
  
  // - Ouvrir le fichier  
  exit_if(!SPIFFS.exists(file), "File doesn't exist");
  File fd = SPIFFS.open(file, FILE_READ, false);
  exit_if(!fd, "Error while opening file");
  print_file_parameters(fd);

  // - lire l'en-tête
  fd.readBytes(signal->header, WAV_HEADER_SIZE);
  // - Extraire la taille
  signal->size = wavefile_length(signal->header);
  Serial.printf("WAV size : %d\n", signal->size);
  // - Allouer un tableau de taille*double
  signal->data = (char *) malloc(signal->size*sizeof(char));
  // - Lire les données
  fd.readBytes(signal->data + WAV_HEADER_SIZE, signal->size - WAV_HEADER_SIZE);
  // - Fermer le fichier
  fd.close();

  Serial.printf("WAV file well opened\n\n");
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

  gauss = (double *) malloc(m * sizeof(double));

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