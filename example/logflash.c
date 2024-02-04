#include "logflash.h"

#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"

page_t *page_a;
page_t *page_b;
page_t *active_page;

log_buffer_header_t g_header;

const char *flash_header_contents = (const char *) (XIP_BASE + FLASH_LOG_HEADER_OFFSET);
const char *flash_target_contents = (const char *) (XIP_BASE + FLASH_LOG_START_OFFSET);

void update_header()
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_LOG_HEADER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_LOG_HEADER_OFFSET, (const uint8_t *) &g_header, FLASH_PAGE_SIZE);
    restore_interrupts (ints);
}

// write to in memory page buffer
// signal DMA when full and switch to other page buffer
void lfprintf(const char *format, ...)
{
    va_list args1, args2, args3;

    // printf the format first
    va_start(args1, format);
    va_copy(args2, args1);
    va_copy(args3, args1);

    // First print to stdout
    vprintf(format, args1);
    va_end(args1);

    // try to write to active buffer
    size_t used = active_page->size;
    char* write_addr = (char *) &(active_page->data) + used;
    size_t remaining = PAGE_DATA_SIZE - used;
#ifdef LOG_H_DEBUG
    printf("page num: %ld\n",g_header.write_page_num);
    printf("used: %d\n",used);
    printf("remaining: %d\n",remaining);
#endif
    int res = vsnprintf(write_addr, remaining, format, args2);
    va_end(args2);
    if (res > remaining)
    {
        *write_addr = '\0'; // cut out the portion of the message written
#ifdef LOG_H_DEBUG
        printf("writing to address: 0x%lx\n",FLASH_LOG_START_OFFSET + g_header.write_page_num * FLASH_PAGE_SIZE);
#endif
        // swap active (for now just dump to flash)
        size_t flash_write_addr =  FLASH_LOG_START_OFFSET + g_header.write_page_num * FLASH_PAGE_SIZE;
        uint32_t ints = save_and_disable_interrupts();
        // at sector boundry erase the nearest sector(4096 bytes - 16 pages) before writing
        if ((flash_write_addr % FLASH_SECTOR_SIZE) == flash_write_addr)
        {
            flash_range_erase(flash_write_addr, FLASH_SECTOR_SIZE);
        }
        flash_range_program(flash_write_addr, (const uint8_t *)active_page, FLASH_PAGE_SIZE);
        restore_interrupts (ints);
#ifdef LOG_H_DEBUG
        printf("write finished\n");
#endif
        active_page->size = 0;
        g_header.write_page_num =  (g_header.write_page_num + 1) % NUM_PAGES;
        res = vsnprintf(active_page->data, PAGE_DATA_SIZE, format, args3);
        update_header();
    }
    va_end(args3);
    active_page->size += res;
    //make sure to 0 terminate
}

void print_all()
{
    const page_t *page;
    // take first page in front of the write pointer (tail of circ buffer)
    int page_num = (g_header.write_page_num + 1) % NUM_PAGES;
    while (page_num != g_header.write_page_num)
    {
#ifdef LOG_H_DEBUG
        printf("testing page %d\n",page_num);
#endif
        page = (page_t *)(flash_target_contents + page_num * FLASH_PAGE_SIZE);
        if (page->magic != PAGE_MAGIC) {
            page_num = (page_num + 1) % NUM_PAGES;
            continue;
        }
#ifdef LOG_H_DEBUG
        printf("Reading page num %d from addr %p, size %ld\n", page_num, page, page->size);
#endif
        printf(page->data);
        page_num = (page_num + 1) % NUM_PAGES;
    }
}

void print_last(int num)
{
    if (num >= NUM_PAGES)
    {
        printf("num should be less than num pages: %d\n", NUM_PAGES);
        return;
    }
    const page_t *page;
    // take first page in front of the write pointer (tail of circ buffer)
    int page_num = (g_header.write_page_num + NUM_PAGES - num) % NUM_PAGES;
    while (page_num != g_header.write_page_num)
    {
#ifdef LOG_H_DEBUG
        printf("testing page %d\n",page_num);
#endif
        page = (page_t *)(flash_target_contents + page_num * FLASH_PAGE_SIZE);
        if (page->magic != PAGE_MAGIC) {
            page_num = (page_num + 1) % NUM_PAGES;
            continue;
        }
#ifdef LOG_H_DEBUG
        printf("Reading page num %d from addr %p, size %ld\n", page_num, page, page->size);
#endif
        printf(page->data);
        page_num = (page_num + 1) % NUM_PAGES;
    }
}

int init_log_flash()
{
    page_a = (page_t *) calloc(sizeof(page_t),1);
    page_b = (page_t *) calloc(sizeof(page_t),1);
    if (page_a == NULL || page_b == NULL){
        printf("failed to allocated page buffer");
        return -1;
    }
    page_a->magic = PAGE_MAGIC;
    page_b->magic = PAGE_MAGIC;
    active_page = page_a;

    g_header = *((log_buffer_header_t *) flash_header_contents);
    if (g_header.magic != LOG_BUFFER_MAGIC)
    {
        //first init
        printf("Initializing flash header\n");
        g_header.magic = LOG_BUFFER_MAGIC;
        g_header.write_page_num = 0;
        update_header();
        
    }
#ifdef LOG_H_DEBUG
    printf("Read flash header, write_page_num: %ld write_page_num: %ld\n", g_header.write_page_num, g_header.write_page_num);
    printf("Active page size: %ld\n",active_page->size);
#endif
    return 0;
}

void erase_all() {
    uint32_t ints = save_and_disable_interrupts();
    // erase 1mB starting at 1mB
    flash_range_erase(FLASH_LOG_HEADER_OFFSET, FLASH_LOG_HEADER_OFFSET);
    restore_interrupts (ints);
    g_header.magic = LOG_BUFFER_MAGIC;
    g_header.write_page_num = 0;
}