#ifndef GUI_FUNC_H_
#define GUI_FUNC_H_

#include "raylib.h"
#include "gui.h"
#include <stdio.h>
#include "my_physics.h"

Vector3 GetVector3FromText(char* text);
void GetTextFromVector3(Vector3 v, char* text);
void GetPhysicsBodyInfo(const PhysicsBody* body, UIState* uiState);
void GetTextFromFloat(float f, char* text);
void ApplyPhysicsBodyTransform(PhysicsBody* body, UIState* uiState);

#endif