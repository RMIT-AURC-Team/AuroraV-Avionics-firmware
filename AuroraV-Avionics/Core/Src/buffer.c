#include "buffer.h"
#include "membuff.h"

MemBuff membuff;
uint8_t buffer_data[16]; 

void initBuff(void) {
    int buffer_size = sizeof(buffer_data);
    int page_size = 4;  
    MemBuff_init(&membuff, buffer_data, buffer_size, page_size);
}
