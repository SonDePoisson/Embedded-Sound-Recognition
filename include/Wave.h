// #ifndef __WAVE_H
// #	define __WAVE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SPIFFS.h>

// Taille de l'en-tête wav
#define WAV_HEADER_SIZE 0x2c

// Position et taille du champs FILESIZE dans l'en-tête
#define WAV_FILESIZE_OFFSET 0x04
#define WAV_FILESIZE_LENGTH 0x04

struct __attribute__((__packed__)) wave_header {
	char file_type_block_id[4]; // 'RIFF'
	uint32_t file_size; // file size - 8
	char file_format_id[4]; // 'WAVE'
};

struct __attribute__((__packed__)) wave_audio_block {
	char format_block_id[4]; // 'fmt '
	uint32_t block_size; //  block size - 8

	uint16_t audio_format; // 1: PCM
	uint16_t channel_count; // 1: MONO, 2: STEREO, ..., 6: CL L C CR R Surround
	uint32_t frequency; // 11025, 22050, 44100
	uint32_t byte_per_sec; // Nombre d'octets à lire par seconde (i.e., Frequence * byte_per_bloc)
	uint16_t byte_per_block; //  Nombre d'octets par bloc d'échantillonnage (i.e., tous canaux confondus : nbr_canaux * bits_per_sample/8).
	uint16_t bits_per_sample; // 8, 16, 24
};

struct __attribute__((__packed__)) wave_data_block_header {
	char data_block_id[4]; // 'DATA'
	uint32_t data_size; // file_size - Header Size (i.e, 44)
};

struct __attribute__((__packed__)) wave_audio_header {
	struct wave_header header;
	struct wave_audio_block audio;
	struct wave_data_block_header data;
};

struct signal {
	char *data;
	long size; 
	char header[WAV_HEADER_SIZE];
};

// #endif

void exit_if(int cond, const char *prefix);

////////// Lecture/Écriture de fichier WAV ///////////


void wavefile_read(char *file, struct signal *signal);

void wavefile_write(char *file, struct signal *signal);

////////// Traitement du signal ///////////

char *gaussienne(char sigma, int *taille);

char convolute(int filter_size, const char *data, const char *filter);

void filter_signal(int size, const char *in, char *out, int filter_size, const char *filter);
