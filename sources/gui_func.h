#ifndef GUI_FUNC_H_
#define GUI_FUNC_H_

#include "raylib.h"
#include "gui.h"
#include <stdio.h>

Vector3 GetVector3FromEditText(char* editText);
void GetEditTextFromVector3(Vector3 v, char* editText);

#endif