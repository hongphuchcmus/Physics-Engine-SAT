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
  Mesh dodecahedron = GenMeshSphere(1, 4, 8); //GenDodecahedron(1.0f);
  Model cubeModel = LoadModelFromMesh(cube);
  Model dodecaModel = LoadModelFromMesh(dodecahedron);
  
  UIState uiState = GuiInit();

  PhysicsBody cubePhysBody = LoadPhysicsBodyFromMesh(cube);
  TextCopy(cubePhysBody.bodyName, "Cube");
  SetColliderType(&cubePhysBody, COLLIDER_TYPE_STATIC);
  cubePhysBody.position = GetVector3FromEditText(uiState.cubePositionEditText);
  PhysicsBody dodecaPhysBody = LoadPhysicsBodyFromMesh(dodecahedron);
  TextCopy(dodecaPhysBody.bodyName, "Dodeca");
  dodecaPhysBody.position = GetVector3FromEditText(uiState.dodecaPositionEditText);
  Vector3 euler = Vector3Scale(GetVector3FromEditText(uiState.cubeRotationEditText), DEG2RAD);
  cubePhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z); 
  euler = Vector3Scale(GetVector3FromEditText(uiState.dodecaRotationEditText), DEG2RAD);
  dodecaPhysBody.rotation = QuaternionFromEuler(euler.x, euler.y, euler.z); 

  physicsTimeScale = 0.0f;

  Manifold lastColInfo = {0};

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
    Manifold colInfo = CheckCollisionPhysicsBodies(&cubePhysBody, &dodecaPhysBody);
    ResolveCollisionPhysicsBodies(&cubePhysBody, &dodecaPhysBody, &colInfo, deltaTime);
    
    if (*uiState.cubePositionEditSubmitted){
      cubePhysBody.position = GetVector3FromEditText(uiState.cubePositionEditText);
    } else if (*uiState.cubePositionEditMode == false) {
      GetEditTextFromVector3(cubePhysBody.position, uiState.cubePositionEditText);
    }
    
    if (*uiState.pauseEnabled == false || *uiState.nextPressed == true){
      lastColInfo = colInfo;
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

    ManifoldToString(&lastColInfo, uiState.collisionInfoText);

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
          DrawModelEx(cubeModel, cubePhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, lastColInfo.overlapping ? DARKGRAY : RED);
        DrawModelWiresEx(cubeModel, cubePhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, ORANGE);
        
        QuaternionToAxisAngle(dodecaPhysBody.rotation, &axisRot, &angleRot);
        angleRot *= RAD2DEG;
        if (!wireframe)
          DrawModelEx(dodecaModel, dodecaPhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, lastColInfo.overlapping ? DARKGRAY : RED);
        DrawModelWiresEx(dodecaModel, dodecaPhysBody.position, axisRot, angleRot, (Vector3){1, 1, 1}, ORANGE);
        
        DrawBoundingBox(PhysicsBodyGetWorldBoundingBoxMargin(&cubePhysBody, 0.01f), lastColInfo.overlapping ? ORANGE : GRAY);
        DrawBoundingBox(PhysicsBodyGetWorldBoundingBoxMargin(&dodecaPhysBody, 0.01f), lastColInfo.overlapping ? ORANGE : GRAY);

        if (lastColInfo.overlapping){
          Vector3 offset = GetCenterOfContactPoints(&lastColInfo);
          //Draw sep axis 
          DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(lastColInfo.seperationAxis), 1000.0f), offset), Vector3Add(Vector3Scale(lastColInfo.seperationAxis, 1000.0f), offset), GREEN);
          // Draw collision points & normals
          for (int32_t i = 0; i <lastColInfo.contactPointCount; i++)
          {
            DrawSphere(lastColInfo.contactPoints[i], 0.1f, GREEN);
            DrawLine3D(lastColInfo.contactPoints[i], Vector3Add(lastColInfo.contactPoints[i], lastColInfo.contactNormals[i]), SKYBLUE);
          }
        } else {
          Vector3 offset = dodecaPhysBody.position;
          DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(lastColInfo.seperationAxis), 1000.0f), offset), Vector3Add(Vector3Scale(lastColInfo.seperationAxis, 1000.0f), offset), PINK);
        }

        // Draw colliding edges
        if (lastColInfo.collisionType == COLLISION_TYPE_EDGE_TO_EDGE){
          DrawSphere(lastColInfo.edge1Start, 0.05f, PURPLE);
          DrawSphere(lastColInfo.edge1End, 0.05f, PURPLE);
          DrawSphere(lastColInfo.edge2Start, 0.05f, PURPLE);
          DrawSphere(lastColInfo.edge2End, 0.05f, PURPLE);

          DrawLine3D(lastColInfo.edge1Start, lastColInfo.edge1End, RED);
          DrawLine3D(lastColInfo.edge2Start, lastColInfo.edge2End, RED);
        }
        DrawGrid(20, 1.0f);
      EndMode3D();
      GuiDraw(renderCommands, uiState);
      DrawFPS(0, 0);
    EndDrawing();
  }
  CloseWindow();
  GuiDeinit(uiState);

  return 0;
}
