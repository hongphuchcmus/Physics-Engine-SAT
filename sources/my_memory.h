#ifndef MY_MEMORY_H_
#define MY_MEMORY_H_
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "raylib.h"
#include <stdalign.h>
#include <memoryapi.h>
#include <stdio.h>

#define MEM_NEW(arena, type, count) (type *)MemArenaAlloc(arena, sizeof(type), alignof(type), count)

typedef struct MemArena {
  char* memory;
  char* nextOffset;
  int32_t capacity;
} MemArena;

MemArena MemArenaNew(int32_t capacity);
void MemArenaClear(MemArena* arena);
void * MemArenaAlloc(MemArena* arena, int32_t size, int32_t align, int32_t count);
void* DebugAlloc(size_t size);
void DebugFree(void* ptr);

#endif