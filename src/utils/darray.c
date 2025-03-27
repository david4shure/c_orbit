#include "darray.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <raylib.h>
#include "logger.h"

darray darray_create(darray arr, int capacity) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));
    header = realloc(header,sizeof(darray_header) + (header->stride * capacity));

    if (!header) return NULL;  // Check for realloc failure

    arr = ((uint8_t*)header + sizeof(darray_header));

    return arr;
}

// Given capacity and size, gives us pointer to the void* array
darray darray_init(int capacity, int stride) {
    // Malloc 32 bits cap, 32 bits size, 32 bits stride + 5 * stride bytes for array
    void* ptr = malloc(sizeof(darray_header) + stride * capacity);

    darray_header* header = (darray_header*) ptr;

    header->size = 0;
    header->stride = stride;
    header->capacity = capacity;

    // Copy data into memory from our struct above
    //memcpy(ptr,header,sizeof(darray_header));

    void* arr = ((uint8_t*)ptr) + sizeof(darray_header);

    // Make ptr point to the beginning of the arr portion of memory rather than the header
    return arr;
}

// Push element onto darray
darray darray_push(darray arr, void* item) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (header->size >= header->capacity) {
        // Increase capacity
        header->capacity *= DARRAY_GROWTH_FACTOR;
        // Realloc new array + header
        arr = darray_create(arr,header->capacity);
        header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));
    }
    void* ptr_to_insert = ((uint8_t*)arr) + header->stride * header->size;

    // Copy the data into our memory
    memcpy(ptr_to_insert, item, header->stride);

    header->size++;

    return arr;
}

// Insert element to array at index
darray darray_insert(darray arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index > header->size) {
        return NULL;
    }

    if (index > header->size-1) {
        // need to REALLOCate
        return NULL;
    }

    if (header->size >= header->capacity) {
        arr = darray_create(arr,header->capacity * DARRAY_GROWTH_FACTOR);
    }

    void* ptr = arr + sizeof(darray_header) + index * header->stride;

    memcpy(ptr,item,header->stride);

    return arr;
}

// Removes item from end of arr
darray darray_pop(darray arr) {
    // Parse out darrray_header from ptr

    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (header->size == 0) {
        return arr;
    }

    void* ptr_to_pop = (uint8_t*)arr + header->stride * (header->size - 1);

    memset(ptr_to_pop, 0, header->stride);

    header->size -= 1;

    return arr;
}

// size = 6
// | 0 | 1 | 2 | 3 | 4 | 5 |
//           ^ item to be removed
//
// memcpy(&2, &3, 6 - 3 * stride);
darray darray_pop_at(darray arr, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (header->size == 0) {
        return arr;
    }

    if (index > header->size - 1) {
        return arr;
    }

    // if index is zero, 5 - 0 + 1, = 6
    size_t size_to_copy = (header->size - 1 - index) * header->stride;

    void* dst = ((uint8_t*)arr) + (header->stride * index);
    void* src = ((uint8_t*)arr) + (header->stride * (index + 1));

    memcpy(dst, src, size_to_copy);

    header->size -= 1;

    return arr;
}

// size = 6
// | 0 | 1 | 2 | 3 | 4 | 5 |
//           ^ insert 10 at index 2
//
// memcpy(&2, &3, 6 - 3 * stride);
// memcpy(&10, &2, header->stride);
darray darray_insert_at(darray arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index < 0) {
        return arr;
    }

    if (header->size + 1 >= header->capacity) {
        header->capacity *= DARRAY_GROWTH_FACTOR;
        // Realloc new array + header
        arr = darray_create(arr,header->capacity);
        header = (darray_header*)(arr - sizeof(darray_header));
    }

    // if index is zero, 5 - 0 + 1, = 6
    size_t size_to_copy = (header->size - index) * header->stride;

    void* dst = ((uint8_t*)arr) + (header->stride * (index + 1));
    void* src = ((uint8_t*)arr) + (header->stride * index);

    // Shift elements to the right by size_to_copy bytes
    memcpy(dst, src, size_to_copy);

    // Copy element into arr at the requested index
    memcpy(src, item, header->stride);

    header->size += 1;

    return arr;
}

// free up our memory starting at header ptr
void darray_free(darray arr) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    free(header);
}

// Insert element to array at index
void* darray_get(darray arr, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index > header->size-1 || index < 0) {
        return NULL;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    return ptr;
}

// Set item in darray at index
void darray_set(darray arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index > header->size-1 || index < 0) {
        return;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    memcpy(ptr,item,header->stride);
}

// Set item in darray at index
void darray_set_length(darray arr, int length) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));
    
    header->size = length; 
}

// Get length of darray
int darray_length(darray arr) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    return header->size;
}

bool darray_empty(darray arr) {
    return darray_length(arr) == 0;
}
