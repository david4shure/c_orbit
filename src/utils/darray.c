#include "darray.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <raylib.h>
#include "logger.h"

void* darray_create(void* arr, int capacity) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));
    header = RL_REALLOC(header,sizeof(darray_header) + (header->stride * capacity));

    arr = ((uint8_t*)header + sizeof(darray_header));

    return arr;
}

// Given capacity and size, gives us pointer to the void* array
void* darray_init(int capacity, int stride) {
    // Malloc 32 bits cap, 32 bits size, 32 bits stride + 5 * stride bytes for array
    void* ptr = RL_MALLOC(sizeof(darray_header) + stride * capacity);

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
void* darray_push(void* arr, void* item) {
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
void* darray_insert(void* arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = arr - sizeof(darray_header);

    if (index > header->size-1) {
        // need to RL_REALLOCate
        Warn("Inserted beyond bounds\n");
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
void* darray_pop(void* arr) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (header->size == 0) {
        Warn("Tried to pop from empty arr");
        return arr;
    }

    void* ptr_to_pop = arr + sizeof(darray_header) * header->stride * (header->size - 1);

    memset(ptr_to_pop, 0, header->stride);

    header->size -= 1;

    return arr;
}

// size = 6
// | 0 | 1 | 2 | 3 | 4 | 5 |
//           ^ item to be removed
//
// memcpy(&2, &3, 6 - 3 * stride);
void* darray_pop_at(void* arr, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (header->size == 0) {
        Warn("Tried to pop from empty arr\n");
        return arr;
    }

    if (index > header->size - 1) {
        Warn("Tried to pop at index > size-1\n");
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
void* darray_insert_at(void* arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index < 0) {
        Warn("Tried to insert out of bound\n");
        return arr;
    }

    if (header->size + 1 >= header->capacity) {
        Debug("Reallocating new array...\n"); // Increase capacity
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
void darray_free(void* arr) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    Debug("free(%p)\n",arr);

    RL_FREE(header);
}

// Insert element to array at index
void* darray_get(void* arr, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index > header->size-1 || index < 0) {
        Warn("Getting beyond bounds\n");
        return NULL;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    return ptr;
}

// Set item in darray at index
void darray_set(void* arr, void* item, int index) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    if (index > header->size-1 || index < 0) {
        Debug("Setting beyond bounds\n");
        return;
    }

    void* ptr = ((uint8_t*)arr) + (index * header->stride);

    memcpy(ptr,item,header->stride);
}

// Get length of darray
int darray_length(void* arr) {
    // Parse out darrray_header from ptr
    darray_header* header = (darray_header*)((uint8_t*)arr - sizeof(darray_header));

    return header->size;
}
