#pragma once

#define DARRAY_DEFAULT_CAPACITY 5
#define DARRAY_GROWTH_FACTOR 2

#include "stdlib.h"
#include "stdbool.h"

// Its a darray under the hood, but helpful to see it as darray type
typedef void* darray;

// MEMORY LAYOUT
// | 32 bits - capacity | 32 bits - size | 32 bits - stride (size of each element) | darray arr
//                                                                                   ^
//                                                                        darr ptr given to caller

// Header struct with list metadata
typedef struct darray_header {
    int capacity; // Total possible elements given the block of memory we've allocated
    int size; // The actual number of elements in the array
    int stride; // the sizeof the type of element stored in this array
} darray_header;

// Creates a new darray
darray darray_create(darray arr, int capacity);

// Given capacity and size, gives us pointer to the darray array
darray darray_init(int capacity, int stride);

// Push item onto the darray, reallocating if necessary
darray darray_push(darray arr, darray item);

// Insert item into array at index, reallocating if necessary
darray darray_insert_at(darray arr, void* item, int index);

// Pop item off of the darray at the end
darray darray_pop(darray arr);

// Pop item off of the darray at index
darray darray_pop_at(darray arr, int index);

// Free up the array
void darray_free(darray arr);

// Print the array
void darray_print(darray arr);

// Get item in array at index
darray darray_get(darray arr, int index);

// Set item in array at index
void darray_set(darray arr, void* item, int index);

bool darray_empty(darray arr);

void darray_set_length(darray arr, int length);

// Length of darray
int darray_length(darray arr);

