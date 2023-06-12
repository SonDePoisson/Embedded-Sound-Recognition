#include <Arduino.h>
#include "Wave.h"

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
  // - Allouer un tableau de taille*char
  signal->data = (char *) malloc(signal->size*sizeof(char));
  // - Lire les données
  fd.readBytes(signal->data + WAV_HEADER_SIZE, signal->size - WAV_HEADER_SIZE);
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
