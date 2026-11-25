/**
 * @file sd_card.h
 * @brief SPI SD-card logging interface.
 */
#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>


void sd_init (void);


bool sd_mount (void);


void sd_unmount (void);


void sd_write_file (const char * p_filename, const char * p_content);


void sd_append_file (const char * p_filename, const char * p_content);


void sd_read_file (const char * p_filename);


#endif /* SD_CARD_H */

/* End of file. */
