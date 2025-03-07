#ifndef GUI_H_
#define GUI_H_
#include "raylib.h"
#include "clay.h"
#include "clay_renderer_raylib.h"
#include "color_def.h"
#include "my_memory.h"
#include "custom_clay.h"
#include "def.h"

// Store info of a selected physics body
typedef struct UIState{
  Font* fonts;
  int32_t fontCount;
  MemArena memArena; // Stores all actual data
  // References to data
  char* name;
  char* position;
  bool* positionEditMode;
  bool* positionSubmitted;
  char* rotation;
  bool* rotationEditMode;
  bool* rotationSubmitted;
  char* velocity;
  bool* velocityEditMode;
  bool* velocitySubmitted;
  char* angularVelocity;
  bool* angularVelocityEditMode;
  bool* angularVelocitySubmitted;
  bool* isPaused;
  bool* nextPressed;
  bool* isFocused;
  bool* restartPressed;
} UIState;

UIState GuiInit();
Clay_RenderCommandArray GuiUpdate(UIState* state);
void GuiDraw(Clay_RenderCommandArray renderCommands, UIState UIState);
void GuiDeinit(UIState state);

#endif