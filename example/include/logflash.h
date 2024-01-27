#ifndef LOGFLASH_
#define LOGFLASH_

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"

// #include "hardware/dma.h"

/*
First page is log circ buffer header

Each page will have page header

*/

#define FLASH_SIZE (2048 * 1024)

#define FLASH_LOG_HEADER_OFFSET (1024 * 1024)
#define FLASH_LOG_START_OFFSET (FLASH_LOG_HEADER_OFFSET + FLASH_SECTOR_SIZE)
#define NUM_PAGES ((FLASH_SIZE - FLASH_LOG_START_OFFSET)/FLASH_PAGE_SIZE)


#define LOG_BUFFER_MAGIC (0xBADF)
#define PAGE_MAGIC (0xFADB)

#define PAGE_DATA_SIZE (232)

// 256 bytes in implicitly reserved since we dedicate whole first flash page
typedef struct log_buffer_header {
    uint32_t magic;
    int size;
    uint32_t write_page_num;
} log_buffer_header_t;

typedef struct page {
    uint32_t magic;
    uint32_t size;
    uint32_t reserved[4]; // in case we need to expand header in the future
    char data[PAGE_DATA_SIZE];
} page_t;

int init_log_flash();

void lfprintf(const char *format, ...);

void print_all();

void erase_all();

//erase function

#endif