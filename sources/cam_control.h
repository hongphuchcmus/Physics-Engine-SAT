#ifndef CAM_CONTROL_H_
#define CAM_CONTROL_H_
#include "raylib.h"

typedef struct CamSettings {
  float moveSpeed;
  float mouseSenstivity;
  float spanSpeed;
  float zoomSpeed;
} CamSettings;

Camera CamInit();
void UpdateCam(Camera* cam, CamSettings* camSettings);

#endif