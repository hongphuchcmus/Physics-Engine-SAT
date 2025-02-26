#include "raylib.h"
#include "gui.h"
#include "gui_func.h"
#include "cam_control.h"
#include "convex_hull.h"
#define CLAY_IMPLEMENTATION
#include "clay.h"
#include "my_physics.h"
#include "memoryapi.h"

#define SCREEN_WIDTH (800)
#define SCREEN_HEIGHT (450)
#define WINDOW_TITLE "Window title"
#define GRAVITY 9.8f


extern float physicsTimeScale;

int main(void)
{
  SetConfigFlags(FLAG_WINDOW_RESIZABLE);
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, WINDOW_TITLE);
  SetTargetFPS(50);
  // Vector3 vertices[20] = {0};
  // CreateRandomVertices(vertices, 20, 0);
  // ConvexShape *convexShape = CreateConvexShape(vertices, 20, 100);
  
  Camera camera = CamInit();
  CamSettings camSettings = {
    .moveSpeed = 10.0f,
    .mouseSenstivity = 0.5f,
    .spanSpeed = 0.1f,
    .zoomSpeed = 2.0f
  };
  
  Mesh cube = GenConvexCube((Vector3){5, 1, 5});
  Mesh dodecahedron = GenMeshSphere(1, 8, 16); //GenDodecahedron(1.0f);
  Model cubeModel = LoadModelFromMesh(cube);
  Model dodecaModel = LoadModelFromMesh(dodecahedron);
  
  UIState uiState = GuiInit();

  PhysicsBody cubePhysBody = LoadPhysicsBodyFromConvexMesh(cube);
  TextCopy(cubePhysBody.bodyName, "Cube");
  cubePhysBody.colliderType = COLLIDER_TYPE_STATIC;
  cubePhysBody.position = GetVector3FromEditText(uiState.cubePositionEditText);
  Vector3 euler = Vector3Scale(GetVector3FromEditText(uiState.cubeRotationEditText), DEG2RAD);
  cubePhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z); 
  
  PhysicsBody dodecaPhysBody = LoadPhysicsBodySphere(1); //LoadPhysicsBodyFromMesh(dodecahedron);
  TextCopy(dodecaPhysBody.bodyName, "Dodeca");
  dodecaPhysBody.position = GetVector3FromEditText(uiState.dodecaPositionEditText);
  euler = Vector3Scale(GetVector3FromEditText(uiState.dodecaRotationEditText), DEG2RAD);
  dodecaPhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z); 
  
  physicsTimeScale = 0.0f;

  Manifold lastManifold = {0};

  SetExitKey(0);
  while (!WindowShouldClose())
  {
    Clay_RenderCommandArray renderCommands = GuiUpdate(&uiState);
    
    if (*uiState.nextPressed == true) {
      physicsTimeScale = 1.0f;
    } else {
      physicsTimeScale = *uiState.pauseEnabled == true ? 0.0f : 1.0f;
    }
    
    float deltaTime = GetFrameTime();
    PhysicsBodyUpdate(&dodecaPhysBody, deltaTime);
    PhysicsBodyUpdate(&cubePhysBody, deltaTime);
    Manifold manifold = CheckCollisionPhysicsBodies(&cubePhysBody, &dodecaPhysBody);
    ResolveCollisionPhysicsBodies(&cubePhysBody, &dodecaPhysBody, &manifold, deltaTime);
    
    if (*uiState.cubePositionEditSubmitted){
      cubePhysBody.position = GetVector3FromEditText(uiState.cubePositionEditText);
    } else if (*uiState.cubePositionEditMode == false) {
      GetEditTextFromVector3(cubePhysBody.position, uiState.cubePositionEditText);
    }
    
    if (*uiState.pauseEnabled == false || *uiState.nextPressed == true){
      lastManifold = manifold;
    }

    if (*uiState.cubeRotationEditSubmitted){
      Vector3 euler = Vector3Scale(GetVector3FromEditText(uiState.cubeRotationEditText), DEG2RAD);
      cubePhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z);
    } else if (*uiState.cubeRotationEditMode == false) {
      Vector3 euler = Vector3Scale(QuaternionToEuler(cubePhysBody.rotation), RAD2DEG);
      GetEditTextFromVector3(euler, uiState.cubeRotationEditText);
    }

    if (*uiState.dodecaPositionEditSubmitted){
      dodecaPhysBody.position = GetVector3FromEditText(uiState.dodecaPositionEditText);
    } else if (*uiState.dodecaPositionEditMode == false) {
      GetEditTextFromVector3(dodecaPhysBody.position, uiState.dodecaPositionEditText);
    }

    if (*uiState.dodecaRotationEditSubmitted){
      Vector3 euler = Vector3Scale(GetVector3FromEditText(uiState.dodecaRotationEditText), DEG2RAD);
      dodecaPhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z);
    } else if (*uiState.dodecaRotationEditMode == false) {
      Vector3 euler = Vector3Scale(QuaternionToEuler(dodecaPhysBody.rotation), RAD2DEG);
      GetEditTextFromVector3(euler, uiState.dodecaRotationEditText);
    }

    // PhysicsBodyToString(&cubePhysBody, uiState.cubePhysicsInfoText);
    // PhysicsBodyToString(&dodecaPhysBody, uiState.dodecaPhysicsInfoText);

    ManifoldToString(&lastManifold, uiState.manifoldText);

    bool wireframe = *uiState.wireframeEnabled; 
    UpdateCam(&camera, &camSettings);
    
    BeginDrawing();
      ClearBackground(RAYWHITE);

      BeginMode3D(camera);
        Vector3 axisRot;
        float angleRot;
        QuaternionToAxisAngle(cubePhysBody.rotation, &axisRot, &angleRot);
        angleRot *= RAD2DEG;
        if (!wireframe)
          DrawModelEx(cubeModel, cubePhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, lastManifold.overlapping ? DARKGRAY : RED);
        DrawModelWiresEx(cubeModel, cubePhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, ORANGE);
        
        QuaternionToAxisAngle(dodecaPhysBody.rotation, &axisRot, &angleRot);
        angleRot *= RAD2DEG;
        if (!wireframe)
          DrawModelEx(dodecaModel, dodecaPhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, lastManifold.overlapping ? DARKGRAY : RED);
        DrawModelWiresEx(dodecaModel, dodecaPhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, ORANGE);
        
        DrawBoundingBox(PhysicsBodyGetWorldBoundingBoxMargin(&cubePhysBody, 0.01f), lastManifold.overlapping ? ORANGE : GRAY);
        DrawBoundingBox(PhysicsBodyGetWorldBoundingBoxMargin(&dodecaPhysBody, 0.01f), lastManifold.overlapping ? ORANGE : GRAY);

        if (lastManifold.overlapping){
          Vector3 offset = GetCenterOfContactPoints(&lastManifold);
          //Draw sep axis 
          DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(lastManifold.seperationAxis), 1000.0f), offset), Vector3Add(Vector3Scale(lastManifold.seperationAxis, 1000.0f), offset), GREEN);
          // Draw collision points & normals
          for (int32_t i = 0; i <lastManifold.contactPointCount; i++)
          {
            DrawSphere(lastManifold.contactPoints[i], 0.1f, GREEN);
            DrawLine3D(lastManifold.contactPoints[i], Vector3Add(lastManifold.contactPoints[i], lastManifold.contactNormals[i]), SKYBLUE);
          }
        } else {
          Vector3 offset = dodecaPhysBody.position;
          DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(lastManifold.seperationAxis), 1000.0f), offset), Vector3Add(Vector3Scale(lastManifold.seperationAxis, 1000.0f), offset), PINK);
        }

        // Draw colliding edges
        if (lastManifold.collisionType == COLLISION_TYPE_EDGE_TO_EDGE){
          DrawSphere(lastManifold.edge1Start, 0.05f, PURPLE);
          DrawSphere(lastManifold.edge1End, 0.05f, PURPLE);
          DrawSphere(lastManifold.edge2Start, 0.05f, PURPLE);
          DrawSphere(lastManifold.edge2End, 0.05f, PURPLE);

          DrawLine3D(lastManifold.edge1Start, lastManifold.edge1End, RED);
          DrawLine3D(lastManifold.edge2Start, lastManifold.edge2End, RED);
        }
        
        DrawPhysicsBodyEdges(&cubePhysBody);
        DrawSphere(lastManifold.nearestEdgeOverlap, 0.1f, YELLOW);
        //Draw axis
        DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(lastManifold.nearestEdgeAxis), 1000.0f), manifold.nearestEdgeOverlap), Vector3Add(Vector3Scale(lastManifold.nearestEdgeAxis, 1000.0f), manifold.nearestEdgeOverlap), BLUE);

        DrawGrid(20, 1.0f);
      EndMode3D();
      GuiDraw(renderCommands, uiState);
      DrawFPS(0, 0);
      DrawPhysicsBodyVertexIndices(&cubePhysBody, camera);
    EndDrawing();
  }
  CloseWindow();
  GuiDeinit(uiState);

  return 0;
}
