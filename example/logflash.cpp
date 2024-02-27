#include "logflash.h"

#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"

#include <string.h>

#include "pico/flash.h"

#include "map.h"

page_t *page_a;
page_t *page_b;
page_t *active_page;

log_buffer_header_t g_header;

const char *flash_header_contents = (const char *) (XIP_BASE + FLASH_LOG_HEADER_OFFSET);
const char *flash_wall_contents = (const char *) (XIP_BASE + FLASH_WALL_START_OFFSET);
const char *flash_target_contents = (const char *) (XIP_BASE + FLASH_LOG_START_OFFSET);


void update_header(void *)
{
#ifdef LOG_H_DEBUG
    printf("writing header write_page:%ld, target: %ld\n",g_header.write_page_num, g_header.target);
#endif

    flash_range_erase(FLASH_LOG_HEADER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_LOG_HEADER_OFFSET, (const uint8_t *) &g_header, FLASH_PAGE_SIZE);
}

void update_target(int target)
{
    g_header.target = target;
    flash_safe_execute(update_header, NULL, 1000);
}

int get_target()
{
    return g_header.target;
}

void write_hwalls_unsafe(int8_t hw[MAZE_HEIGHT - 1][MAZE_WIDTH])
{
    flash_range_erase(FLASH_WALL_START_OFFSET, FLASH_SECTOR_SIZE);
    h_wall_buffer h;
    h.magic = WALLS_MAGIC;
    memcpy(h.h_walls,hw, (MAZE_HEIGHT-1)*MAZE_WIDTH);
    flash_range_program(FLASH_WALL_START_OFFSET, (const uint8_t *) &h, FLASH_PAGE_SIZE);
}

void write_vwalls_unsafe(int8_t vw[MAZE_HEIGHT][MAZE_WIDTH - 1])
{
    v_wall_buffer v;
    v.magic = WALLS_MAGIC;
    memcpy(v.v_walls,vw, MAZE_HEIGHT*(MAZE_WIDTH-1));
    flash_range_program(FLASH_WALL_START_OFFSET + FLASH_PAGE_SIZE, (const uint8_t *) &v, FLASH_PAGE_SIZE);
}

void write_walls(int8_t *h_walls, int8_t *v_walls)
{
    flash_safe_execute((void (*)(void *))write_hwalls_unsafe, (void *)h_walls, 1000);
    flash_safe_execute((void (*)(void *))write_vwalls_unsafe, (void *)v_walls, 1000);
}

int read_walls(int8_t h_walls[MAZE_HEIGHT - 1][MAZE_WIDTH], int8_t v_walls[MAZE_HEIGHT][MAZE_WIDTH - 1])
{
    h_wall_buffer_t *h = (h_wall_buffer_t *) (flash_wall_contents);
    if (h->magic != WALLS_MAGIC)
    {
        return -1;
    }
    v_wall_buffer_t *v = (v_wall_buffer_t *) (flash_wall_contents + FLASH_PAGE_SIZE);
    if (v->magic != WALLS_MAGIC)
    {
        return -2;
    }
    memcpy(h_walls, h->h_walls, (MAZE_HEIGHT - 1) * MAZE_WIDTH);
    memcpy(v_walls, v->v_walls, MAZE_HEIGHT * (MAZE_WIDTH-1));

    return 0;

}

void write_flash(void *)
{
    // swap active (for now just dump to flash)
    size_t flash_write_addr =  FLASH_LOG_START_OFFSET + g_header.write_page_num * FLASH_PAGE_SIZE;
    // at sector boundry erase the nearest sector(4096 bytes - 16 pages) before writing
    if ((flash_write_addr % FLASH_SECTOR_SIZE) == 0)
    {
        flash_range_erase((flash_write_addr / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
    }
    flash_range_program(flash_write_addr, (const uint8_t *)active_page, FLASH_PAGE_SIZE);
    g_header.write_page_num =  (g_header.write_page_num + 1) % NUM_PAGES;
    update_header(NULL);
}

void write_flash_safe()
{
    flash_safe_execute(write_flash,NULL, 1000);
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
    int remaining = PAGE_DATA_SIZE - used - 1;
#ifdef LOG_H_DEBUG
    printf("page num: %ld\n",g_header.write_page_num);
    printf("used: %d\n",used);
    printf("remaining: %d\n",remaining);
#endif
    int res = vsnprintf(write_addr, remaining, format, args2);
    va_end(args2);
    if (res >= remaining)
    {
        *write_addr = '\0'; // cut out the portion of the message written
#ifdef LOG_H_DEBUG
        printf("writing to address: %ld, 0x%lx\n",g_header.write_page_num, FLASH_LOG_START_OFFSET + g_header.write_page_num * FLASH_PAGE_SIZE);
#endif
        flash_safe_execute(write_flash,NULL, 1000);
#ifdef LOG_H_DEBUG
        printf("write finished\n");
#endif
        active_page->size = 0;
        res = vsnprintf(active_page->data, PAGE_DATA_SIZE, format, args3);
    }
    active_page->size += res;
    va_end(args3);
    //make sure to 0 terminate
}

void print_all()
{
    const page_t *page;
    // take first page in front of the write pointer (tail of circ buffer)
    uint32_t page_num = (g_header.write_page_num + 1) % NUM_PAGES;
    while (page_num != g_header.write_page_num)
    {
#ifdef LOG_H_DEBUG
        printf("testing page %ld\n",page_num);
#endif
        page = (page_t *)(flash_target_contents + page_num * FLASH_PAGE_SIZE);
        if (page->magic != PAGE_MAGIC) {
            page_num = (page_num + 1) % NUM_PAGES;
            continue;
        }
#ifdef LOG_H_DEBUG
        printf("Reading page num %ld from addr %p, size %ld\n", page_num, page, page->size);
#endif
        printf(page->data);
        page_num = (page_num + 1) % NUM_PAGES;
    }
}

void print_last(uint32_t num)
{
    if (num >= NUM_PAGES)
    {
        printf("num should be less than num pages: %d\n", NUM_PAGES);
        return;
    }
    const page_t *page;
    // take first page in front of the write pointer (tail of circ buffer)
    uint32_t page_num = (g_header.write_page_num + NUM_PAGES - num) % NUM_PAGES;
    while (page_num != g_header.write_page_num)
    {
#ifdef LOG_H_DEBUG
        printf("testing page %ld\n",page_num);
#endif
        page = (page_t *)(flash_target_contents + page_num * FLASH_PAGE_SIZE);
        if (page->magic != PAGE_MAGIC) {
            page_num = (page_num + 1) % NUM_PAGES;
            continue;
        }
#ifdef LOG_H_DEBUG
        printf("Reading page num %ld from addr %p, size %ld\n", page_num, page, page->size);
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
        flash_safe_execute(update_header, NULL, 1000);
        
    }
#ifdef LOG_H_DEBUG
    printf("Read flash header, write_page_num: %ld write_page_num: %ld\n", g_header.write_page_num, g_header.write_page_num);
    printf("Active page size: %ld\n",active_page->size);
#endif
    return 0;
}

void erase_all() {
    // erase 1mB starting at 1mB
    flash_range_erase(FLASH_LOG_HEADER_OFFSET, FLASH_LOG_HEADER_OFFSET);
    g_header.magic = LOG_BUFFER_MAGIC;
    g_header.write_page_num = 0;
    g_header.target = -1;
}