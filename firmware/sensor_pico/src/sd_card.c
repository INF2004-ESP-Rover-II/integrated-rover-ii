#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ff.h"
#include <stdio.h>   // ← Add this
#include <string.h> 

#define CS_PIN   15
#define SCK_PIN  10
#define MOSI_PIN 12
#define MISO_PIN 11

FATFS fs;
FIL file;
FRESULT fr;

// 1. SD card hardware init
void sd_init() {
    spi_init(spi1, 400*1000);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);
    sleep_ms(1000);
}

// 2. Mount filesystem
bool sd_mount() {
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("Mount error: %d\n", fr);
        return false;
    }
    return true;
}

// 3. Create/write file (overwrite)
void sd_write_file(const char* filename, const char* content) {
    fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        UINT bw;
        f_write(&file, content, strlen(content), &bw);
        f_close(&file);
    }
}

// 4. Append to file
void sd_append_file(const char* filename, const char* content) {
    fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr == FR_OK) {
        UINT bw;
        f_write(&file, content, strlen(content), &bw);
        f_close(&file);
    }
}

// 5. Read file
void sd_read_file(const char* filename) {
    fr = f_open(&file, filename, FA_READ);
    if (fr == FR_OK) {
        char buf[64];
        printf("Printing lines in file:\n");
        while (f_gets(buf, sizeof(buf), &file)) {
            printf("%s", buf);
        }
        f_close(&file);
    }
}

// 6. Unmount
void sd_unmount() {
    f_mount(NULL, "", 1);
}