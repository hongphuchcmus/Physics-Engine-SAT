#ifndef GUI_H_
#define GUI_H_
#include "raylib.h"
#include "clay.h"
#include "clay_renderer_raylib.h"
#include "color_def.h"
#include "my_memory.h"
#include "custom_clay.h"
#include "def.h"

typedef struct UIState{
  Font* fonts;
  int32_t fontCount;
  MemArena memArena; // Stores all actual data
  // References to data
  char* cubePositionEditText;
  char* dodecaPositionEditText;
  bool* cubePositionEditMode;
  bool* dodecaPositionEditMode;
  
  char* cubeRotationEditText;
  char* dodecaRotationEditText;
  bool* cubeRotationEditMode;
  bool* dodecaRotationEditMode;
  
  bool* cubePositionEditSubmitted;
  bool* dodecaPositionEditSubmitted;
  bool* cubeRotationEditSubmitted;
  bool* dodecaRotationEditSubmitted;
  
  bool* wireframeEnabled;
  bool* wireframeCheckToggled;
  
  bool* prevPressed;
  bool* nextPressed;
  bool* pauseToggled;
  bool* pauseEnabled;

  char* manifoldText;
  char* cubePhysicsInfoText;
  char* dodecaPhysicsInfoText;
} UIState;

UIState GuiInit();
Clay_RenderCommandArray GuiUpdate(UIState* state);
void GuiDraw(Clay_RenderCommandArray renderCommands, UIState UIState);
void GuiDeinit(UIState state);

#endif