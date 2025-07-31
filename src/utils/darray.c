#include "darray.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <raylib.h>
#include "logger.h"

// Helper function to get header pointer from darray
static darray_header* get_header(darray arr) {
    size_t header_size = sizeof(darray_header);
    size_t aligned_offset = (header_size + 7) & ~7; // Round up to 8-byte boundary
    return (darray_header*)((uint8_t*)arr - aligned_offset);
}

darray darray_create(darray arr, int capacity) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);
    
    // For reallocation, we need to handle alignment carefully
    size_t header_size = sizeof(darray_header);
    size_t aligned_offset = (header_size + 7) & ~7; // Round up to 8-byte boundary
    size_t data_size = header->stride * capacity;
    size_t new_size = aligned_offset + data_size;
    new_size = (new_size + 7) & ~7; // Round up to 8-byte boundary
    
    void* new_ptr = realloc(header, new_size);

    if (!new_ptr) return NULL;  // Check for realloc failure

    header = (darray_header*)new_ptr;
    header->capacity = capacity; // Update capacity
    arr = ((uint8_t*)header + aligned_offset);

    return arr;
}

// Given capacity and size, gives us pointer to the void* array
darray darray_init(int capacity, int stride) {
    // Ensure stride is 8-byte aligned
    stride = (stride + 7) & ~7;
    
    // Calculate aligned offset for the data section
    size_t header_size = sizeof(darray_header);
    size_t aligned_offset = (header_size + 7) & ~7; // Round up to 8-byte boundary
    
    // Calculate total size and ensure it's a multiple of 8 for aligned_alloc
    size_t data_size = stride * capacity;
    size_t total_size = aligned_offset + data_size;
    total_size = (total_size + 7) & ~7; // Round up to 8-byte boundary
    
    // Use aligned allocation for 8-byte alignment
    void* ptr = aligned_alloc(8, total_size);
    
    if (!ptr) {
        // Fallback to regular malloc if aligned_alloc fails
        ptr = malloc(total_size);
        if (!ptr) return NULL;
    }

    darray_header* header = (darray_header*) ptr;

    header->size = 0;
    header->stride = stride;
    header->capacity = capacity;
    
    void* arr = ((uint8_t*)ptr) + aligned_offset;

    // Make ptr point to the beginning of the arr portion of memory rather than the header
    return arr;
}

// Push element onto darray
darray darray_push(darray arr, void* item) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);

    if (header->size >= header->capacity) {
        // Increase capacity
        header->capacity *= DARRAY_GROWTH_FACTOR;
        // Realloc new array + header
        arr = darray_create(arr,header->capacity);
        header = get_header(arr);
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
    darray_header* header = get_header(arr);

    if (index > header->size) {
        return NULL;
    }

    if (index > header->size-1) {
        // need to REALLOCate
        return NULL;
    }

    if (header->size >= header->capacity) {
        arr = darray_create(arr,header->capacity * DARRAY_GROWTH_FACTOR);
        header = get_header(arr);
    }

    void* ptr = arr + index * header->stride;

    memcpy(ptr,item,header->stride);

    return arr;
}

// Removes item from end of arr
darray darray_pop(darray arr) {
    // Parse out darrray_header from ptr

    darray_header* header = get_header(arr);

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
    darray_header* header = get_header(arr);

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
    darray_header* header = get_header(arr);

    if (index < 0) {
        return arr;
    }

    if (header->size + 1 >= header->capacity) {
        header->capacity *= DARRAY_GROWTH_FACTOR;
        // Realloc new array + header
        arr = darray_create(arr,header->capacity);
        header = get_header(arr);
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
    darray_header* header = get_header(arr);

    free(header);
}

// Insert element to array at index
void* darray_get(darray arr, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);

    if (index > header->size-1 || index < 0) {
        return NULL;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    return ptr;
}

// Set item in darray at index
void darray_set(darray arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);

    if (index > header->size-1 || index < 0) {
        return;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    memcpy(ptr,item,header->stride);
}

// Set item in darray at index
void darray_set_length(darray arr, int length) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);
    
    header->size = length; 
}

// Get length of darray
int darray_length(darray arr) {
    // Parse out darrray_header from ptr
    darray_header* header = get_header(arr);

    return header->size;
}

bool darray_empty(darray arr) {
    return darray_length(arr) == 0;
}
