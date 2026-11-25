/**
 * @file sd_card.c
 * @brief Sd card firmware module.
 */
#include "sd_card.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ff.h"
#include <stdio.h> // ← Add this
#include <string.h>

#define CS_PIN 15
#define SCK_PIN 10
#define MOSI_PIN 12
#define MISO_PIN 11

static FATFS   g_fs;
static FIL     g_file;
static FRESULT g_fr;

// 1. SD card hardware init
void
sd_init (void)
{
    spi_init(spi1, 400 * 1000);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);
    sleep_ms(1000);
}

// 2. Mount filesystem
bool
sd_mount (void)
{
    g_fr = f_mount(&g_fs, "", 1);
    if (g_fr != FR_OK)
    {
        printf("Mount error: %d\n", g_fr);
        return false;
    }
    return true;
}

// 3. Create/write file (overwrite)
void
sd_write_file (const char * p_filename, const char * p_content)
{

    g_fr = f_open(&g_file, p_filename,
                  (uint32_t) (FA_WRITE) | (uint32_t) (FA_CREATE_ALWAYS));

    if (g_fr == FR_OK)
    {
        UINT bytes_written = 0;
        f_write(&g_file, p_content, strlen(p_content), &bytes_written);
        f_close(&g_file);
    }
}

// 4. Append to file
void
sd_append_file (const char * p_filename, const char * p_content)
{

    g_fr = f_open(&g_file, p_filename,
                  (uint32_t) (FA_WRITE) | (uint32_t) (FA_OPEN_APPEND));

    if (g_fr == FR_OK)
    {
        UINT bytes_written = 0;
        f_write(&g_file, p_content, strlen(p_content), &bytes_written);
        f_close(&g_file);
    }
}

// 5. Read file
void
sd_read_file (const char * p_filename)
{
    g_fr = f_open(&g_file, p_filename, FA_READ);
    if (g_fr == FR_OK)
    {
        char buf[64];
        printf("Printing lines in file:\n");
        while (f_gets(buf, sizeof(buf), &g_file))
        {
            printf("%s", buf);
        }
        f_close(&g_file);
    }
}

// 6. Unmount
void
sd_unmount (void)
{
    f_mount(NULL, "", 1);
}

/* End of file. */
