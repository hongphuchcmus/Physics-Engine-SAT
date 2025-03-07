#include "gui.h"

static void f_HandleClayErrors(Clay_ErrorData errorData){
  TraceLog(LOG_ERROR, errorData.errorText.chars);
}

UIState GuiInit()
{
  UIState uiState = (UIState) {
    .memArena = MemArenaNew(1024),
  };
  uiState.fontCount = 4;
  uiState.fonts = MEM_NEW(&uiState.memArena, Font, uiState.fontCount);
  
  uiState.name = MEM_NEW(&uiState.memArena, char, 100);
  uiState.position = MEM_NEW(&uiState.memArena, char, 100);
  uiState.rotation = MEM_NEW(&uiState.memArena, char, 100);
  uiState.velocity = MEM_NEW(&uiState.memArena, char, 100);
  uiState.angularVelocity = MEM_NEW(&uiState.memArena, char, 100);
  uiState.isPaused = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.nextPressed = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.isFocused = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.positionEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.rotationEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.velocityEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.angularVelocityEditMode = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.positionSubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.rotationSubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.velocitySubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.angularVelocitySubmitted = MEM_NEW(&uiState.memArena, bool, 1);
  uiState.restartPressed = MEM_NEW(&uiState.memArena, bool, 1);

  uint64_t clayRequiredMemory = Clay_MinMemorySize();
  Clay_Arena clayMemory = Clay_CreateArenaWithCapacityAndMemory(clayRequiredMemory, malloc(clayRequiredMemory));

  Clay_Initialize(clayMemory, (Clay_Dimensions){
    .width =  GetScreenWidth(),
    .height = GetScreenHeight()
  }, (Clay_ErrorHandler){f_HandleClayErrors});

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
            .childGap = 8,
            .childAlignment = {.x = CLAY_ALIGN_X_LEFT},
            .sizing = {
              CLAY_SIZING_FIXED(240),
              CLAY_SIZING_FIT(0)
            },
            .padding = CLAY_PADDING_ALL(8),
          },
          .backgroundColor = COLOR_WHITE,
          .cornerRadius = CLAY_CORNER_RADIUS(4),
          .border = {
            .width = { .left = 1, .right = 1, .top = 1, .bottom = 1 },
            .color = COLOR_GRAY_800
          }
      }){
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){
          *uiState->isFocused = Clay_Hovered();
        }

        // Header
        CLAY({
          .layout = {
            .sizing = {
              .width = CLAY_SIZING_GROW(0),
              .height = CLAY_SIZING_FIT(0)
            },
            .layoutDirection = CLAY_TOP_TO_BOTTOM,
            .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
            .childGap = 8,
            .padding = CLAY_PADDING_ALL(4)
          },
        }){
          CLAY_TEXT(CLAY_STRING("Physics Body Info"), CLAY_TEXT_CONFIG({
            .fontId = FONT_BOLD_ID,
            .fontSize = 16,
            .textColor = COLOR_BLACK
          }));
          // Name
          Clay_String nameClayText = CustomClay_ToClayString(uiState->name);
          CLAY_TEXT(nameClayText, CLAY_TEXT_CONFIG({
            .fontSize = 16,
            .textColor = COLOR_BLACK
          }));
        }
        
        // Position
        CLAY_TEXT(CLAY_STRING("Position"), CLAY_TEXT_CONFIG({
          .fontId = FONT_BOLD_ID,
          .fontSize = 16,
          .textColor = COLOR_BLACK
        }));
        *uiState->positionSubmitted = CustomClay_TextBox((Clay_Sizing) {
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->position, 100, uiState->positionEditMode);
        
        // Rotation
        CLAY_TEXT(CLAY_STRING("Rotation"), CLAY_TEXT_CONFIG({
          .fontId = FONT_BOLD_ID,
          .fontSize = 16,
          .textColor = COLOR_BLACK
        }));
        *uiState->rotationSubmitted = CustomClay_TextBox((Clay_Sizing) {
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->rotation, 100, uiState->rotationEditMode);

        // Velocity
        CLAY_TEXT(CLAY_STRING("Velocity"), CLAY_TEXT_CONFIG({
          .fontId = FONT_BOLD_ID,
          .fontSize = 16,
          .textColor = COLOR_BLACK
        }));
        *uiState->velocitySubmitted = CustomClay_TextBox((Clay_Sizing) {
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->velocity, 100, uiState->velocityEditMode);

        // Angular Velocity
        CLAY_TEXT(CLAY_STRING("Angular Velocity"), CLAY_TEXT_CONFIG({
          .fontId = FONT_BOLD_ID,
          .fontSize = 16,
          .textColor = COLOR_BLACK
        }));
        *uiState->angularVelocitySubmitted = CustomClay_TextBox((Clay_Sizing) {
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, uiState->angularVelocity, 100, uiState->angularVelocityEditMode);
        
        CLAY({
          .layout = {
            .sizing = {.width = CLAY_SIZING_GROW(0), .height = CLAY_SIZING_GROW(0)},
            .layoutDirection = CLAY_LEFT_TO_RIGHT,
            .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
            .childGap = 4,
            .padding = CLAY_PADDING_ALL(4)
          }
        }){
          CustomClay_Toggle((Clay_Sizing){
            .width = CLAY_SIZING_GROW(0),
            .height = CLAY_SIZING_FIT(0)
          }, "Paused", "Running", uiState->isPaused);
          CustomClay_Button((Clay_Sizing){
            .width = CLAY_SIZING_FIT(0),
            .height = CLAY_SIZING_FIT(0)
          }, "Next", uiState->nextPressed);
        };
        *uiState->restartPressed = CustomClay_Button((Clay_Sizing){
          .width = CLAY_SIZING_GROW(0),
          .height = CLAY_SIZING_FIT(0)
        }, "Restart", uiState->restartPressed);
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
