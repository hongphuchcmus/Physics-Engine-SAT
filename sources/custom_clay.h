#ifndef CUSTOM_CLAY_H_
#define CUSTOM_CLAY_H_

#include "raylib.h"
#include "clay.h"
#include "my_memory.h"
#include "color_def.h"
#include "def.h"

#define TEXTBOX_EDIT_MAX_LENGTH 256

bool CustomClay_CheckBox(Clay_Sizing claySizing, char* text, bool* isChecked);
bool CustomClay_TextBox(Clay_Sizing claySizing, char* text, int maxLength,  bool* editMode);
void CustomClay_Toggle(Clay_Sizing claySizing, const char *pressedText, const char *unpressedText, bool* isPressed);
bool CustomClay_Button(Clay_Sizing claySizing, const char* text, bool* isPressed);
Clay_String CustomClay_ToClayString(char* charArray);

#endif