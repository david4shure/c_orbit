#pragma once
#include <stdlib.h>

#define DARRAY_DEFAULT_CAPACITY 5
#define DARRAY_GROWTH_FACTOR 2

// MEMORY LAYOUT
// | 32 bits - capacity | 32 bits - size | 32 bits - stride (size of each element) | void* arr
//                                                                                   ^
//                                                                        darr ptr given to caller

// Header struct with list metadata
typedef struct darray_header {
    int capacity; // Total possible elements given the block of memory we've allocated
    int size; // The actual number of elements in the array
    int stride; // the sizeof the type of element stored in this array
} darray_header;

// Creates a new darray
void* darray_create(void* arr, int capacity);

// Given capacity and size, gives us pointer to the void* array
void* darray_init(int capacity, int stride);

// Push item onto the darray, reallocating if necessary
void* darray_push(void* arr, void* item);

// Insert item into array at index, reallocating if necessary
void* darray_insert_at(void* arr, void* item, int index);

// Pop item off of the darray at the end
void* darray_pop(void* arr);

// Pop item off of the darray at index
void* darray_pop_at(void* arr, int index);

// Free up the array
void darray_free(void* arr);

// Print the array
void darray_print(void* arr);

// Get item in array at index
void* darray_get(void* arr, int index);

// Set item in array at index
void darray_set(void* arr, void* item, int index);

// Length of darray
int darray_length(void* arr);

