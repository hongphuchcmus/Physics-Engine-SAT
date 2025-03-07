#include "my_memory.h"

MemArena MemArenaNew(int32_t capacity){
  MemArena memArena = {0};
  memArena.memory = (char*)MemAlloc(capacity);
  memArena.nextOffset = memArena.memory;
  memArena.capacity = capacity;
  return memArena;
}

void MemArenaClear(MemArena* arena){
  if (arena->memory != 0){
    MemFree(arena->memory);
  }
  arena->memory = 0;
  arena->capacity = 0;
  arena->nextOffset = 0;
}

void * MemArenaAlloc(MemArena* arena, int32_t size, int32_t align, int32_t count){
  int32_t padding = -((int32_t)arena->nextOffset & (align - 1));
  int32_t available = arena->capacity - (int32_t)(arena->nextOffset - arena->memory) - padding;
  if (available < 0 || size * count > available){
    return NULL;
  }
  void* p = arena->nextOffset + padding;
  arena->nextOffset = (char*)p + size * count;
  return memset(p, 0, size * count);
}

void MemSet(void* memory, int32_t value, int32_t size) {
  memset(memory, value, size);
}
