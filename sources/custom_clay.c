#include "custom_clay.h"

void ClayCustom__HandleTextBoxInteraction(Clay_ElementId elementId, Clay_PointerData pointerInfo, intptr_t editMode)
{
  if (editMode == 0)
  {
    return;
  }
  if (pointerInfo.state == CLAY_POINTER_DATA_PRESSED_THIS_FRAME)
  {
    *((bool *)editMode) = true;
  }
}

void ClayCustom__HandleButtonInteraction(Clay_ElementId elementId, Clay_PointerData pointerInfo, intptr_t isPressed)
{
  if (isPressed == 0)
  {
    return;
  }
  if (pointerInfo.state == CLAY_POINTER_DATA_PRESSED_THIS_FRAME)
  {
    *((bool *)isPressed) = true;
  }
}

void ClayCustom__HandleToggleInteraction(Clay_ElementId elementId, Clay_PointerData pointerInfo, intptr_t isPressed)
{
  if (isPressed == 0)
  {
    return;
  }
  if (pointerInfo.state == CLAY_POINTER_DATA_PRESSED_THIS_FRAME)
  {
    *((bool *)isPressed) = !*((bool *)isPressed);
  }
}

void ClayCustom__HandleCheckBoxInteraction(Clay_ElementId elementId, Clay_PointerData pointerInfo, intptr_t isChecked)
{
  if (isChecked == 0)
  {
    return;
  }
  if (pointerInfo.state == CLAY_POINTER_DATA_PRESSED_THIS_FRAME)
  {
    *((bool *)isChecked) = !(*((bool *)isChecked));
  }
}

// Return true when the text box is submitted
bool CustomClay_TextBox(Clay_Sizing claySizing, char *text, int maxLength, bool *editMode)
{
  if (maxLength > TEXTBOX_EDIT_MAX_LENGTH)
  {
    maxLength = TEXTBOX_EDIT_MAX_LENGTH;
  }
  // This is a hacky way to do caret: duplicating the text,
  // then draw it twice, one for the full text, one for the text
  // upto the current caret position
  // Two text must have the same color or else the text with caret
  // always on top

  static char textEditBuffer[TEXTBOX_EDIT_MAX_LENGTH] = {0};
  static float nextBackSpaceDeletion = 0;
  static float nextLeftCaretNavigation = 0;
  static float nextRightCaretNavigation = 0;
  static int currentCaretPosition = 0;

  // Update Edit Box Content
  if (*editMode == true)
  {
    int32_t textLength = TextLength(text);
    TextCopy(textEditBuffer, text);
    // Add an extra character for the caret when appending text
    int32_t textEditBufferLength = textLength + 1;
    textEditBuffer[textEditBufferLength] = '\0';
    float time = GetTime();
    if (textLength >= 1 && currentCaretPosition > 0 && (IsKeyPressed(KEY_BACKSPACE) || (IsKeyDown(KEY_BACKSPACE) && time > nextBackSpaceDeletion))){
      // Delete character before caret
      for (int i = currentCaretPosition; i < textLength; i++)
      {
        text[i - 1] = text[i];
        textEditBuffer[i - 1] = text[i];
      }
      textLength--;
      textEditBufferLength--;
      text[textLength] = '\0';
      textEditBuffer[textEditBufferLength] = '\0';
      currentCaretPosition--;
      if (currentCaretPosition < 0)
      {
        currentCaretPosition = 0;
      }
      if (IsKeyPressed(KEY_BACKSPACE))
      {
        nextBackSpaceDeletion = time + 0.3f;
      } else {
        nextBackSpaceDeletion = time + 0.05f;
      }
    } else if (IsKeyPressed(KEY_LEFT) || (time > nextLeftCaretNavigation && IsKeyDown(KEY_LEFT)))
    {
      currentCaretPosition--;
      if (currentCaretPosition < 0)
      {
        currentCaretPosition = 0;
      }
      if (IsKeyPressed(KEY_LEFT)){
        nextLeftCaretNavigation = time + 0.3f;
      } else {
        nextLeftCaretNavigation = time + 0.05f;
      }
    }
    else if (IsKeyPressed(KEY_RIGHT) || (time > nextRightCaretNavigation && IsKeyDown(KEY_RIGHT)))
    {
      currentCaretPosition++;
      if (currentCaretPosition > textLength)
      {
        currentCaretPosition = textLength;
      }
      if (IsKeyPressed(KEY_RIGHT)){
        nextRightCaretNavigation = time + 0.3f;
      } else {
        nextRightCaretNavigation = time + 0.05f;
      }
    } else {
      int pressedChar = GetCharPressed();
      while (pressedChar != 0 && textLength < maxLength)
      { 
        // Shift characters after the caret to the right
        for (int i = textLength; i > currentCaretPosition; i--)
        {
          text[i] = text[i - 1];
          textEditBuffer[i] = textEditBuffer[i-1];
        }
        // Insert current character
        text[currentCaretPosition] = (char)pressedChar;
        textEditBuffer[currentCaretPosition] = (char)pressedChar;
        // Increase text lengths and shift caret position
        textLength++;
        textEditBufferLength++;
        currentCaretPosition++;
        text[textLength] = '\0';
        textEditBuffer[textEditBufferLength] = '\0';
        pressedChar = GetCharPressed();
      }
    }
    
    if ((int)(GetTime() * 100) % 50 >= 45)
    {
      textEditBuffer[currentCaretPosition] = ' ';
    }
    else
    {
      textEditBuffer[currentCaretPosition] = '_';
    }

    textEditBuffer[currentCaretPosition+1] = '\0'; 
  }

  bool clickedOut = false;
  bool clickedIn = false;

  CLAY({
    .backgroundColor = (Clay_Hovered() || *editMode == true) ? (Clay_Color)COLOR_GRAY_50 : (Clay_Color) COLOR_GRAY_100,
    .layout = {
      .sizing = claySizing,
      .padding = CLAY_PADDING_ALL(4) // Text padding
    },
    .cornerRadius = CLAY_CORNER_RADIUS(4),
    .border = {.width = {.left = 1, .right = 1, .top = 1, .bottom = 1}, .color = COLOR_GRAY_900}})
  {
    clickedOut = (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) && !Clay_Hovered();
    clickedIn = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && Clay_Hovered();
    Clay_OnHover(ClayCustom__HandleTextBoxInteraction, (intptr_t)editMode);
    Clay_String clayText = CustomClay_ToClayString(text);
    if (clayText.length > 0)
    {
      CLAY_TEXT(clayText, CLAY_TEXT_CONFIG({.fontSize = 16, .fontId = FONT_REGULAR_ID, .textColor = COLOR_BLACK}));
    }
    else
    {
      CLAY_TEXT(CLAY_STRING("..."), CLAY_TEXT_CONFIG({.fontSize = 16, .fontId = FONT_REGULAR_ID, .textColor = COLOR_GRAY_500}));
    }
    if (*editMode == true){
      CLAY({
        .layout = {
          .layoutDirection = CLAY_TOP_TO_BOTTOM,
          .sizing = {
            .width = CLAY_SIZING_GROW(0),
            .height = CLAY_SIZING_GROW(0)
          },
          .padding = CLAY_PADDING_ALL(4)
        },
        .floating = {
          .attachTo = CLAY_ATTACH_TO_PARENT,
          .attachPoints = {
            .element = CLAY_ATTACH_POINT_LEFT_TOP,
            .parent = CLAY_ATTACH_POINT_LEFT_TOP
          },
        }
      }){
        Clay_String clayText = CustomClay_ToClayString(textEditBuffer);
        CLAY_TEXT(clayText, CLAY_TEXT_CONFIG({.fontSize = 16, .fontId = FONT_REGULAR_ID, .textColor = COLOR_BLACK}));
      }
    }
  }

  if (clickedIn)
  {
    *editMode = true;
    currentCaretPosition = TextLength(text);
    return false;
  }
  else if (*editMode && (clickedOut || IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_ESCAPE)))
  {
    *editMode = false;
    return true;
  }
  return false;
}

bool CustomClay_Button(Clay_Sizing claySizing, const char *text, bool* isPressed)
{
  if (isPressed == 0)
  {
    return false;
  }
  *(bool *)isPressed = false;
  bool clicked = false;
  CLAY({
    .layout = {
      .childAlignment = {
        .x = CLAY_ALIGN_X_CENTER,
        .y = CLAY_ALIGN_Y_CENTER
      },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .padding = {8, 8, 4, 4},
      .sizing = claySizing},
    .backgroundColor = Clay_Hovered() ? (IsMouseButtonDown(MOUSE_BUTTON_LEFT) ? (Clay_Color)COLOR_GRAY_100 : (Clay_Color)COLOR_GRAY_50) : (Clay_Color)COLOR_GRAY_200,
    .border = {.width = {1, 1, 1, 1}, .color = COLOR_GRAY_900},
    .cornerRadius = {4, 4, 4, 4},
  })
  {
    clicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && Clay_Hovered();
    Clay_String clayText = CustomClay_ToClayString(text);
    //Clay_OnHover(ClayCustom__HandleButtonInteraction, (intptr_t)isPressed);
    CLAY_TEXT(clayText, CLAY_TEXT_CONFIG({.fontSize = 16, .fontId = FONT_BOLD_ID, .textColor = Clay_Hovered() ? (Clay_Color)COLOR_GRAY_500 : (Clay_Color)COLOR_BLACK}));
  }
  *isPressed = clicked;
  return clicked;
}


bool CustomClay_Toggle(Clay_Sizing claySizing, const char *pressedText, const char *unpressedText, bool* isPressed)
{
  if (isPressed == 0)
  {
    return false;
  }
  const char* text = *isPressed == true ? pressedText : unpressedText;
  CLAY({
    .layout = {
      .childAlignment = {
        .x = CLAY_ALIGN_X_CENTER,
        .y = CLAY_ALIGN_Y_CENTER
      },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .padding = {8, 8, 4, 4},
      .sizing = claySizing},
    .backgroundColor = (*isPressed) ? (Clay_Hovered() ? (Clay_Color)COLOR_GRAY_200 : (Clay_Color)COLOR_GRAY_300) : (Clay_Hovered() ? (Clay_Color)COLOR_GRAY_50 : (Clay_Color)COLOR_GRAY_100),
    .border = {.width = {1, 1, 1, 1}, .color = (*isPressed) ? (Clay_Color)COLOR_GRAY_800 : (Clay_Color)COLOR_GRAY_900},
    .cornerRadius = {4, 4, 4, 4},
  })
  {
    Clay_String clayText = CustomClay_ToClayString((char*)text);
    Clay_OnHover(ClayCustom__HandleToggleInteraction, (intptr_t)isPressed);
    CLAY_TEXT(clayText, CLAY_TEXT_CONFIG({.fontSize = 16, .fontId = FONT_BOLD_ID, .textColor = (*isPressed) ? (Clay_Color)COLOR_GRAY_800 : (Clay_Color)COLOR_BLACK}));
  }
}

Clay_String CustomClay_ToClayString(char *charArray)
{
  return (Clay_String){
      .length = TextLength(charArray),
      .chars = charArray};
}

bool CustomClay_CheckBox(Clay_Sizing claySizing, char* text, bool* isChecked){
  bool clicked = false;
  
  CLAY({
    .layout = {
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
      .sizing = claySizing,
      .padding = CLAY_PADDING_ALL(4),
      .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}
    },
  }){
    // CheckBox
    CLAY({
      .layout = {
        .sizing = {
          .width = CLAY_SIZING_FIXED(20),
          .height = CLAY_SIZING_FIXED(20)
        },
        .childAlignment = {
          .x = CLAY_ALIGN_X_CENTER,
          .y = CLAY_ALIGN_Y_CENTER
        }
      },
      .cornerRadius = {4, 4, 4, 4},
      .border = {
        .width = {1, 1, 1, 1},
        .color = COLOR_GRAY_800
      },
      .backgroundColor = Clay_Hovered() ? (Clay_Color) COLOR_GRAY_100 : (Clay_Color) COLOR_GRAY_50
    }){
      clicked = IsMouseButtonDown(MOUSE_BUTTON_LEFT) && Clay_Hovered();
      Clay_OnHover(ClayCustom__HandleCheckBoxInteraction, (intptr_t)isChecked);
      if (*isChecked == true){
        CLAY({
          .layout = {
            .sizing = {
              .width = CLAY_SIZING_FIXED(10),
              .height = CLAY_SIZING_FIXED(10)
            },
            .childAlignment = {
              .x = CLAY_ALIGN_X_CENTER,
              .y = CLAY_ALIGN_Y_CENTER
            }
          },
          .cornerRadius = {4, 4, 4, 4},
          .border = {
            .width = {1, 1, 1, 1},
            .color = COLOR_CYAN_800
          },
          .backgroundColor = COLOR_CYAN_800
        }){}
      }
    }
    CLAY({
      .layout = {
        .sizing = {
          .width = CLAY_SIZING_FIXED(4),
          .height = CLAY_SIZING_GROW(0) 
        }
      }
    }){};
    Clay_String clayText = CustomClay_ToClayString(text);
    CLAY_TEXT(clayText, CLAY_TEXT_CONFIG({
      .fontSize = 16,
      .textColor = COLOR_GRAY_900
    }));
  }
  return clicked;
}
