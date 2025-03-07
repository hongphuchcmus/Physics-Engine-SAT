#include "gui_func.h"

Vector3 GetVector3FromText(char *text)
{
  Vector3 position = {0};
  sscanf(text, "%f %f %f", &position.x, &position.y, &position.z);
  return position;
}

void GetTextFromVector3(Vector3 v, char* text)
{
  sprintf(text, "%.3g %.3g %.3g", v.x, v.y, v.z);
}

// void GetPhysicsBodyInfo(const PhysicsBody* body, UIState* uiState) {
//   TextCopy(uiState->name, body->bodyName);
//   GetTextFromVector3(body->position, uiState->position);
//   GetTextFromVector3(Vector3Scale(QuaternionToEuler(body->rotation), RAD2DEG), uiState->rotation);
//   GetTextFromVector3(body->velocity, uiState->velocity);
//   GetTextFromVector3(body->angularVelocity, uiState->angularVelocity);
// }

void GetTextFromFloat(float f, char* text) {
  sprintf(text, "%.3g", f);
}

void ApplyPhysicsBodyTransformFromUIState(PhysicsBody* body, UIState* uiState) {
  if (*uiState->positionSubmitted) {
    body->position = GetVector3FromText(uiState->position);
  }
  if (*uiState->rotationSubmitted) {
    Vector3 eulerRotation = Vector3Scale(GetVector3FromText(uiState->rotation), DEG2RAD);
    body->rotation = QuaternionFromEuler(eulerRotation.x, eulerRotation.y, eulerRotation.z);
  }
  if (*uiState->velocitySubmitted) {
    body->velocity = GetVector3FromText(uiState->velocity);
  }
  if (*uiState->angularVelocitySubmitted) {
    body->angularVelocity = GetVector3FromText(uiState->angularVelocity);
  }
}

void UpdateUIStateFromPhysicsBody(const PhysicsBody* body, UIState* uiState) {
  if (!*uiState->positionEditMode){
    GetTextFromVector3(body->position, uiState->position);
  }
  if (!*uiState->rotationEditMode){
    GetTextFromVector3(Vector3Scale(QuaternionToEuler(body->rotation), RAD2DEG), uiState->rotation);
  }
  if (!*uiState->velocityEditMode){
    GetTextFromVector3(body->velocity, uiState->velocity);
  }
  if (!*uiState->angularVelocityEditMode){
    GetTextFromVector3(body->angularVelocity, uiState->angularVelocity);
  }
}
