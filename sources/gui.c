#include "gui.h"

static void fHandleClayErrors(Clay_ErrorData errorData){
  TraceLog(LOG_ERROR, errorData.errorText.chars);
}

UIState GuiInit()
{
  UIState uiState = (UIState) {
    .memArena = MemArenaNew(1024),
  };
  uiState.fontCount = 4;
  uiState.fonts = MEM_NEW(&uiState.memArena, Font, uiState.fontCount);
  
  uiState.cubePositionEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.cubePositionEditSubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.cubePositionEditText = MEM_NEW(&uiState.memArena, char, 100);
  
  uiState.dodecaPositionEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.dodecaPositionEditSubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.dodecaPositionEditText = MEM_NEW(&uiState.memArena, char, 100);

  uiState.cubeRotationEditText = MEM_NEW(&uiState.memArena, char, 100);
  uiState.cubeRotationEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.cubeRotationEditSubmitted = MEM_NEW(&uiState.memArena, bool, 1);

  uiState.collisionInfoText = MEM_NEW(&uiState.memArena, char, 256);

  uiState.dodecaRotationEditText = MEM_NEW(&uiState.memArena, char, 100);
  uiState.dodecaRotationEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.dodecaRotationEditSubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  
  uiState.prevPressed = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.nextPressed = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.wireframeEnabled = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.wireframeCheckToggled = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.pauseEnabled = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.pauseToggled = MEM_NEW(&uiState.memArena, bool, 1);

  //uiState.cubePhysicsInfoText = MEM_NEW(&uiState.memArena, char, 256);
  //uiState.dodecaPhysicsInfoText = MEM_NEW(&uiState.memArena, char, 256);
  
  TextCopy(uiState.cubePositionEditText, "1 0 1");
  TextCopy(uiState.cubeRotationEditText, "20 0 0");
  TextCopy(uiState.dodecaPositionEditText, "1 4 1"); 
  TextCopy(uiState.dodecaRotationEditText, "0 0 10");
  *uiState.pauseEnabled = true;

  uint64_t clayRequiredMemory = Clay_MinMemorySize();
  Clay_Arena clayMemory = Clay_CreateArenaWithCapacityAndMemory(clayRequiredMemory, malloc(clayRequiredMemory));

  Clay_Initialize(clayMemory, (Clay_Dimensions){
    .width =  GetScreenWidth(),
    .height = GetScreenHeight()
  }, (Clay_ErrorHandler){fHandleClayErrors});

  uiState.fonts[FONT_DEFAULT_ID] = LoadFontEx(FONT_DEFAULT, 16, 0, 400);
  uiState.fonts[FONT_MONO_ID] = LoadFontEx(FONT_MONO, 16, 0, 400);
  uiState.fonts[FONT_BOLD_ID] = LoadFontEx(FONT_BOLD, 16, 0, 400);
  uiState.fonts[FONT_REGULAR_ID] = LoadFontEx(FONT_REGULAR, 16, 0, 400);
  SetTextureFilter(uiState.fonts[0].texture, TEXTURE_FILTER_BILINEAR);
  SetTextureFilter(uiState.fonts[1].texture, TEXTURE_FILTER_BILINEAR);
  Clay_SetMeasureTextFunction(Raylib_MeasureText, uiState.fonts);
  Clay_SetDebugModeEnabled(false);

  return uiState;
}

Clay_RenderCommandArray 
GuiUpdate(UIState* uiState)
{
  Clay_SetLayoutDimensions((Clay_Dimensions) {
    .width = GetScreenWidth(),
    .height = GetScreenHeight()
  });

  Vector2 mousePosition = GetMousePosition(); 
  Clay_SetPointerState((Clay_Vector2) {
    .x = mousePosition.x,
    .y = mousePosition.y
  }, IsMouseButtonDown(MOUSE_BUTTON_LEFT));
  Vector2 mouseWheelMov = GetMouseWheelMoveV();

  Clay_UpdateScrollContainers(
    true,
    (Clay_Vector2) {
      mouseWheelMov.x,
      mouseWheelMov.y
    },
    GetFrameTime()
  );

  Clay_BeginLayout();
    CLAY({
      .backgroundColor = COLOR_TRANSPARENT,
      .layout = {
        .layoutDirection = CLAY_TOP_TO_BOTTOM,
        .sizing = {
          CLAY_SIZING_GROW(0),
          CLAY_SIZING_GROW(0)
        },
        .childAlignment = {.x = CLAY_ALIGN_X_RIGHT},
        .padding = CLAY_PADDING_ALL(4),
        .childGap = 4,
      },
    }){
      CLAY({
          .layout = {
            .layoutDirection = CLAY_TOP_TO_BOTTOM,
            .childGap = 10,
            .childAlignment = {.x = CLAY_ALIGN_X_LEFT},
            .sizing = {
              CLAY_SIZING_FIXED(280),
              CLAY_SIZING_FIT(0)
            },
            .padding = {8, 8, 8, 16},
          },
          .backgroundColor = COLOR_WHITE,
          .cornerRadius = CLAY_CORNER_RADIUS(4),
          .border = {
            .width = { .left = 1, .right = 1, .top = 1, .bottom = 1 },
            .color = COLOR_GRAY_800
          }
      }){
        CLAY_TEXT(CLAY_STRING("Cube Position"), CLAY_TEXT_CONFIG({
          .fontSize = 16,
          .textColor = {0, 0, 0, 255} 
        }));
        *uiState->cubePositionEditSubmitted = false;
        if (CustomClay_TextBox((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->cubePositionEditText, 100, uiState->cubePositionEditMode)){
          *uiState->cubePositionEditSubmitted = true;
        }
        CLAY_TEXT(CLAY_STRING("Cube Rotation"), CLAY_TEXT_CONFIG({
          .fontSize = 16,
          .textColor = {0, 0, 0, 255} 
        }));
        *uiState->cubeRotationEditSubmitted = false;
        if (CustomClay_TextBox((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->cubeRotationEditText, 100, uiState->cubeRotationEditMode)){
          *uiState->cubeRotationEditSubmitted = true;
        }
        *uiState->dodecaPositionEditSubmitted = false;
        CLAY_TEXT(CLAY_STRING("Dodecahedron Position"), CLAY_TEXT_CONFIG({
          .fontSize = 16,
          .textColor = {0, 0, 0, 255} 
        }));
        if (CustomClay_TextBox((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->dodecaPositionEditText, 100, uiState->dodecaPositionEditMode)){
          *uiState->dodecaPositionEditSubmitted = true;
        }
        *uiState->dodecaRotationEditSubmitted = false;
        CLAY_TEXT(CLAY_STRING("Dodecahedron Rotation"), CLAY_TEXT_CONFIG({
          .fontSize = 16,
          .textColor = {0, 0, 0, 255} 
        }));
        if (CustomClay_TextBox((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->dodecaRotationEditText, 100, uiState->dodecaRotationEditMode)){
          *uiState->dodecaRotationEditSubmitted = true;
        }
        if (CustomClay_CheckBox((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, "Wireframe", uiState->wireframeEnabled)){
          *uiState->wireframeCheckToggled = true;
        }
        CLAY({
          .layout = {
            .sizing = {
              .width = CLAY_SIZING_GROW(0),
              .height = CLAY_SIZING_FIT(0)
            },
            .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
            .childGap = 4,
            .padding = CLAY_PADDING_ALL(4)
          },
        }){
          // CustomClay_Button((Clay_Sizing){
          //   .width = CLAY_SIZING_FIT(0),
          //   .height = CLAY_SIZING_GROW(0)
          // }, "Prev", uiState->prevPressed);
          
          *uiState->pauseToggled = false;
          if (CustomClay_Toggle((Clay_Sizing){
            .width = CLAY_SIZING_GROW(0),
            .height = CLAY_SIZING_FIT(0)
          }, "Paused", "Pause", uiState->pauseEnabled)){
            *uiState->pauseToggled = true;
          }
          CustomClay_Button((Clay_Sizing){
            .width = CLAY_SIZING_FIT(0),
            .height = CLAY_SIZING_GROW(0)
          }, "Next", uiState->nextPressed);
        }
        Clay_String colInfoClayText = CustomClay_ToClayString(uiState->collisionInfoText);
        CLAY_TEXT(colInfoClayText, CLAY_TEXT_CONFIG({
          .fontSize = 16,
          .textColor = COLOR_BLACK
        }));
        // Clay_String cubePhysicsInfoClayText = CustomClay_ToClayString(uiState->cubePhysicsInfoText);
        // CLAY_TEXT(cubePhysicsInfoClayText, CLAY_TEXT_CONFIG({
        //   .fontSize = 16,
        //   .textColor = COLOR_BLACK
        // }));
        // Clay_String dodecaPhysicsInfoClayText = CustomClay_ToClayString(uiState->dodecaPhysicsInfoText);
        // CLAY_TEXT(dodecaPhysicsInfoClayText, CLAY_TEXT_CONFIG({
        //   .fontSize = 16,
        //   .textColor = COLOR_BLACK
        // }));
        
      }
    }
  return Clay_EndLayout();
}

void GuiDraw(Clay_RenderCommandArray renderCommands, UIState uiState)
{
  Clay_Raylib_Render(renderCommands, uiState.fonts);
}

void GuiDeinit(UIState uiState)
{
  for (int32_t i = 0; i < uiState.fontCount; i++)
  {
    UnloadFont(uiState.fonts[i]);
  }
  
  MemArenaClear(&uiState.memArena);
}
