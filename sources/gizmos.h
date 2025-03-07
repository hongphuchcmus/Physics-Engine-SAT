#ifndef GIZMOS_H_
#define GIZMOS_H_
#include "my_physics.h"
#include "color_def.h"
#include "geometry.h"

#define GIZMOS_TRANSLATION_NONE 0
#define GIZMOS_TRANSLATION_PLANE 1
#define GIZMOS_TRANSLATION_Y 2

typedef struct GizmosState {
  Camera camera;
  PhysicsBody* selectedBody;
  Vector3 translation;
  int32_t translationMode;
} GizmosState;

GizmosState GizmosInit();
void GizmosUpdate(GizmosState* gizmosState, Camera camera, PhysicsBody* physicsBodies, int32_t PhysicsBodyCount);
void GizmosDraw(const GizmosState* gizmosState, Camera camera);

#endif