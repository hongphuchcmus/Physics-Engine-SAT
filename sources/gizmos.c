#include "gizmos.h"

static int32_t f_SelectPhysicsBody(PhysicsBody* physicsBodies, int32_t physicsBodyCount, Camera camera) {
  Ray camRay = GetScreenToWorldRay(GetMousePosition(), camera);

  RayCollision* rayCollisions = MemAlloc(physicsBodyCount * sizeof(RayCollision));
  for (int32_t i = 0; i < physicsBodyCount; i++){
    rayCollisions[i] = GetRayCollisionBox(camRay, PhysicsBodyGetWorldBoundingBoxMargin(physicsBodies + i, 0.01f));
  }

  float minDistance = 1e30f;
  int32_t minDistanceIdx = -1;
  for (int32_t i = 0; i < physicsBodyCount; i++)
  {
    if (rayCollisions[i].hit){
      if (minDistance > rayCollisions[i].distance){
        minDistance = rayCollisions[i].distance;
        minDistanceIdx = i;
      }
    }
  }

  if (minDistanceIdx < 0){
    return -1;
  }
  
  return minDistanceIdx;
}

static Vector3 f_GetScreenToYPlane(Vector2 screenPos, Vector3 pointPlane, Camera camera){
  Ray camRay = GetScreenToWorldRay(screenPos, camera);
  float dPlane = -Vector3DotProduct(pointPlane, (Vector3){0, 1, 0});
  float t = Vector3DotProduct((Vector3){0, 1, 0}, camRay.position) + dPlane;
  t /= -Vector3DotProduct((Vector3){0, 1, 0}, camRay.direction);
  return Vector3Add(camRay.position, Vector3Scale(camRay.direction, t));
}

static Vector3 f_PhysicsBodyManipulate(PhysicsBody* physicsBody, Camera camera, Vector3 offset, int32_t* outTranslationMode) {
  Vector2 mouseDelta = GetMouseDelta();
  if (Vector2LengthSqr(mouseDelta) < 4.0f){
    return (Vector3) {0};
  }
  if (IsKeyDown(KEY_LEFT_ALT)){
    physicsBody->position.y -= mouseDelta.y * 0.1f;
    *outTranslationMode = GIZMOS_TRANSLATION_Y;
  } else {
    *outTranslationMode = GIZMOS_TRANSLATION_PLANE;
    float y = physicsBody->position.y;
    Vector3 mousePointOnPlane = f_GetScreenToYPlane(GetMousePosition(), physicsBody->position, camera);
    mousePointOnPlane = Vector3Add(mousePointOnPlane, offset);
    Vector3 translation = Vector3Subtract(mousePointOnPlane, physicsBody->position);
    physicsBody->position = mousePointOnPlane;
    physicsBody->position.y = y;
    return translation;
  }
}

GizmosState GizmosInit() {
  GizmosState gizmosState = {0};
  return gizmosState;
}

void GizmosUpdate(GizmosState* gizmosState, Camera camera, PhysicsBody* physicsBodies, int32_t physicsBodyCount) {
  static Vector3 offsetToSelected = {0};
  
  int32_t selected = -1;
  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){
    selected = f_SelectPhysicsBody(physicsBodies, physicsBodyCount, camera);
    if (selected >= 0){
      gizmosState->selectedBody = physicsBodies + selected;
      Vector3 mousePointOnPlane = f_GetScreenToYPlane(GetMousePosition(), physicsBodies[selected].position, camera);
      offsetToSelected = Vector3Subtract(physicsBodies[selected].position, mousePointOnPlane);
    } else {
      gizmosState->selectedBody = NULL;
      offsetToSelected = (Vector3){0};
    }
  }

  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
    if (gizmosState->selectedBody != NULL){
      gizmosState->translation = f_PhysicsBodyManipulate(gizmosState->selectedBody, camera, offsetToSelected, &gizmosState->translationMode);
    }
  } else {
    gizmosState->translation = (Vector3){0};
    gizmosState->translationMode = GIZMOS_TRANSLATION_NONE;
  }
}

void GizmosDraw(const GizmosState* gizmosState, Camera camera) {
  if (gizmosState->selectedBody != NULL){
    BoundingBox box = PhysicsBodyGetWorldBoundingBox(gizmosState->selectedBody);
    DrawBoundingBox(GetBoundingBoxMargin(box, 0.02f), (Color) COLOR_ORANGE_700);
    DrawBoundingBox(GetBoundingBoxMargin(box, 0.04f), (Color) COLOR_ORANGE_800);
    DrawBoundingBox(GetBoundingBoxMargin(box, 0.06f), (Color) COLOR_ORANGE_700);
    
    if (gizmosState->translationMode == GIZMOS_TRANSLATION_Y){
      DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate((Vector3){0, 1, 0}), 1000.0f), gizmosState->selectedBody->position), Vector3Add(Vector3Scale((Vector3){0, 1, 0}, 1000.0f), gizmosState->selectedBody->position), (Color) COLOR_GRAY_100);
    } else if (gizmosState->translationMode == GIZMOS_TRANSLATION_PLANE){
      DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate((Vector3){1, 0, 0}), 1000.0f), gizmosState->selectedBody->position), Vector3Add(Vector3Scale((Vector3){1, 0, 0}, 1000.0f), gizmosState->selectedBody->position), (Color) COLOR_GRAY_200);
      DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate((Vector3){0, 0, 1}), 1000.0f), gizmosState->selectedBody->position), Vector3Add(Vector3Scale((Vector3){0, 0, 1}, 1000.0f), gizmosState->selectedBody->position), (Color) COLOR_GRAY_200);
      DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate((Vector3){0, 1, 0}), 1000.0f), gizmosState->selectedBody->position), Vector3Add(Vector3Scale((Vector3){0, 1, 0}, 1000.0f), gizmosState->selectedBody->position), (Color) COLOR_GRAY_100);
    }
  }
}
