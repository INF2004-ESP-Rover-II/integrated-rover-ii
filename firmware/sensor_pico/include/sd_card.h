#ifndef FATFS_SPI_H
#define FATFS_SPI_H

#include <stdint.h>
#include "ff.h" // FatFS support

// Set your SPI pin configuration here
#define CS_PIN    15
#define SCK_PIN   10
#define MOSI_PIN  11
#define MISO_PIN  12

// SPI/SD initialization function
void sd_init(void);

// File system mount and unmount
FRESULT sd_mount();
FRESULT sd_unmount();

// File operations
FRESULT sd_write_file(const char *filename, const char *content);
FRESULT sd_append_file(const char *filename, const char *content);
FRESULT sd_read_file(const char *filename);

#endif
