#include <Arduino.h>
#include "Wave.h"

// Fonctions utilisé pour la lecture du signal
/**
 * Extract `size` bytes as little endian value from `data` (starting at `offset`)
 * Return the native representation.
 */
long long extract_le2n(int size, int16_t data[], int offset)
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
unsigned long wavefile_length(int16_t header[])
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

void wavefile_read(char *file, struct signal *signal)
{ 
  exit_if(!SPIFFS.begin(), "An Error has occurred while mounting SPIFFS");

  // - Ouvrir le fichier  
  exit_if(!SPIFFS.exists(file), "File doesn't exist");
  File fd = SPIFFS.open(file, FILE_READ, false);
  exit_if(!fd, "Error while opening file");
  print_file_parameters(fd);

  // - lire l'en-tête
  char buffer_header[WAV_HEADER_SIZE];
  fd.readBytes(buffer_header, WAV_HEADER_SIZE);
  for (size_t i = 0; i < WAV_HEADER_SIZE; i++)
    signal->header[i] = buffer_header[i];
  
  // - Extraire la taille
  signal->size = wavefile_length(signal->header);
  Serial.printf("WAV size : %d\n", signal->size);
  // - Allouer un tableau de taille*char
  signal->data = (int16_t *) malloc(signal->size*sizeof(int16_t));
  Serial.printf("Allocated data\n");
  // - Lire les données
  int16_t *buffer_data;
  buffer_data = (int16_t *) malloc(signal->size*sizeof(int16_t));
  int idx = 0;
  while (fd.available())
  {
    buffer_data[idx] = fd.read();
    idx++;
  }
  for (size_t i = 0; i < signal->size; i+=2)
    signal->data[i] = buffer_data[i]<<8|buffer_data[i+1];
  
  // - Fermer le fichier
  fd.close();

  Serial.printf("WAV file well opened\n\n");
}

void wavefile_write(char *file, struct signal *signal)
{
  exit_if(!SPIFFS.begin(), "An Error has occurred while mounting SPIFFS");

  // - Ouvrir le fichier  
  exit_if(!SPIFFS.exists(file), "File doesn't exist");
  File fd = SPIFFS.open(file, FILE_WRITE);
  exit_if(!fd, "Error while opening file");

  fd.write((uint8_t *) signal->data, signal->size);
}
