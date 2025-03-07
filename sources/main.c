#include "raylib.h"
#include "gui.h"
#include "gui_func.h"
#include "cam_control.h"
#include "convex_hull.h"
#define CLAY_IMPLEMENTATION
#include "clay.h"
#include "my_physics.h"
#include "my_memory.h"
#include "gizmos.h"

#define SCREEN_WIDTH (800)
#define SCREEN_HEIGHT (450)
#define WINDOW_TITLE "Window title"

extern float physicsTimeScale;

int main(void)
{
  //physicsTimeScale = 0.2f;
  
  SetConfigFlags(FLAG_WINDOW_RESIZABLE);
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, WINDOW_TITLE);
  SetTargetFPS(60);

  Camera camera = CamInit();
  CamSettings camSettings = {
    .moveSpeed = 10.0f,
    .mouseSenstivity = 0.5f,
    .spanSpeed = 0.1f,
    .zoomSpeed = 2.0f
  };
  

  int32_t physicsBodyCount = 5;
  PhysicsBody* physicsBodies = MemAlloc(physicsBodyCount * sizeof(PhysicsBody));
  MemSet(physicsBodies, 0, physicsBodyCount * sizeof(PhysicsBody));
  Model* models = MemAlloc(physicsBodyCount * sizeof(Model));
  Color colors[] = {
    (Color) COLOR_RED_600,
    (Color) COLOR_GREEN_600,
    (Color) COLOR_BLUE_600,
    (Color) COLOR_YELLOW_600,
    (Color) COLOR_PURPLE_600,
    (Color) COLOR_RED_800,
    (Color) COLOR_GREEN_800,
    (Color) COLOR_BLUE_800,
    (Color) COLOR_YELLOW_800,
    (Color) COLOR_PURPLE_800
  };
  
  PhysicsBodyTransform physicsBodyInitialTransforms[] = {
    (PhysicsBodyTransform) {
      .position = (Vector3){0, 0, 0},
      .velocity = (Vector3){0, 0, 0},
      .angularVelocity = (Vector3){0, 0, 0},
      .rotation = QuaternionIdentity()
    },
    (PhysicsBodyTransform) {
      .position = (Vector3){-2.5f, 1.5f, -3},
      .velocity = (Vector3){0, 0, 0},
      .angularVelocity = (Vector3){0, 0, 0},
      .rotation = QuaternionIdentity()
    },
    (PhysicsBodyTransform) {
      .position = (Vector3){0.5f, 1.0f, -2.5f},
      .velocity = (Vector3){0, 0, 0},
      .angularVelocity = (Vector3){0, 0, 0},
      .rotation = QuaternionIdentity()
    },
    (PhysicsBodyTransform) {
      .position = (Vector3){0, 5.0f, -0.8f},
      .velocity = (Vector3){0, 0, 0},
      .angularVelocity = (Vector3){0, 0, 0},
      .rotation = QuaternionIdentity()
    },
    (PhysicsBodyTransform) {
      .position = (Vector3){0, 5.7f, -0.8f},
      .velocity = (Vector3){0, 0, 0},
      .angularVelocity = (Vector3){0, 0, 0},
      .rotation = QuaternionIdentity()
    }
  };

  models[0] = LoadModelFromMesh(GenMeshCube(8, 1, 8));
  physicsBodies[0] = LoadPhysicsBodyFromConvexMesh(models[0].meshes[0]);
  physicsBodies[0].colliderType = COLLIDER_TYPE_STATIC;
  TextCopy(physicsBodies[0].bodyName, "static cube 0");
  
  models[1] = LoadModelFromMesh(GenMeshCube(3, 2, 2));
  physicsBodies[1] = LoadPhysicsBodyFromConvexMesh(models[1].meshes[0]);
  physicsBodies[1].colliderType = COLLIDER_TYPE_STATIC;
  TextCopy(physicsBodies[1].bodyName, "static cube 1");

  models[2] = LoadModelFromMesh(GenMeshCube(3, 1, 3));
  physicsBodies[2] = LoadPhysicsBodyFromConvexMesh(models[2].meshes[0]);
  physicsBodies[2].colliderType = COLLIDER_TYPE_STATIC;
  TextCopy(physicsBodies[2].bodyName, "static cube 2");
  
  models[3] = LoadModelFromMesh(GenMeshSphere(1, 4, 8));
  physicsBodies[3] = LoadPhysicsBodyFromConvexMesh(models[3].meshes[0]);
  physicsBodies[3].colliderType = COLLIDER_TYPE_RIGID;
  TextCopy(physicsBodies[3].bodyName, "rigid convex 3");

  models[4] = LoadModelFromMesh(GenMeshSphere(1.0f, 6, 12));
  physicsBodies[4] = LoadPhysicsBodySphere(1.0f);
  physicsBodies[4].colliderType = COLLIDER_TYPE_RIGID;
  TextCopy(physicsBodies[4].bodyName, "rigid sphere 4");

  for (int32_t i = 0; i < physicsBodyCount; i++){
    PhysicsBodyReplaceTransform(physicsBodies + i, physicsBodyInitialTransforms[i]);
  }

  Manifold* manifolds = MemAlloc(20 * sizeof(Manifold));
  int32_t manifoldCount = 0;

  UIState uiState = GuiInit();
  GizmosState gizmosState = GizmosInit();

  SetExitKey(0);

  *uiState.isPaused = true;
  while (!WindowShouldClose())
  {
    Clay_RenderCommandArray commandArray = GuiUpdate(&uiState);

    if (*uiState.isPaused){
      physicsTimeScale = 0.0f;
    } else {
      physicsTimeScale = 1.0f;
    }
    if (*uiState.isPaused && *uiState.nextPressed){
      physicsTimeScale = 1.0f;
    }
    
    int32_t selected = -1;
    if (!*uiState.isFocused){
      GizmosUpdate(&gizmosState, camera, physicsBodies, physicsBodyCount);
    }
    if (gizmosState.selectedBody){
      GetPhysicsBodyInfo(gizmosState.selectedBody, &uiState);
    }
    ApplyPhysicsBodyTransformFromUIState(gizmosState.selectedBody, &uiState);
    if (*uiState.restartPressed) {
      for (int32_t i = 0; i < physicsBodyCount; i++) {
        PhysicsBodyReplaceTransform(physicsBodies + i, physicsBodyInitialTransforms[i]);
      }
    }

    // Physics
    manifoldCount = 0;
    for (int32_t i = 0; i < physicsBodyCount - 1; i++) {
      for (int32_t j = i + 1; j < physicsBodyCount; j++) {
        Manifold manifold = CheckCollisionPhysicsBodies(physicsBodies + i, physicsBodies + j);
        if (manifold.overlapping) {
          ResolveCollisionPhysicsBodies(physicsBodies + i, physicsBodies + j, &manifold, GetFrameTime());
        }
        manifolds[manifoldCount++] = manifold;
      }
    }

    for (int32_t i = 0; i < physicsBodyCount; i++)
    {
      PhysicsBodyApplyGravity(physicsBodies + i, GetFrameTime());
      PhysicsBodyUpdate(physicsBodies + i, GetFrameTime());
    }

    ////

    UpdateCam(&camera, &camSettings);

    BeginDrawing();
      ClearBackground((Color)COLOR_GRAY_600);

      BeginMode3D(camera);
        for (int32_t i = 0; i < physicsBodyCount; i++)
        {
          Vector3 axis; float angle;
          QuaternionToAxisAngle(physicsBodies[i].rotation, &axis, &angle);
          DrawModelEx(models[i], physicsBodies[i].position,
            axis, angle * RAD2DEG, (Vector3){1, 1, 1}, (Color) colors[i % 5]);
          DrawModelWiresEx(models[i], physicsBodies[i].position,
            axis, angle * RAD2DEG, (Vector3){1, 1, 1}, ColorBrightness(colors[i % 5], -0.3f));
        }
        // Debug manifolds
        for (int32_t i = 0; i < manifoldCount; i++)
        {
          if (manifolds[i].overlapping){
            for (int32_t j = 0; j < manifolds[i].contactPointCount; j++)
            {
              DrawSphere(manifolds[i].contactPoints[j], 0.05f, (Color) COLOR_CYAN_500);
              DrawLine3D(manifolds[i].contactPoints[j], Vector3Add(manifolds[i].contactPoints[j], Vector3Scale(manifolds[i].contactNormals[j], 0.5f)), (Color) COLOR_GREEN_400);
            }
          }
        }
        //DrawGrid(20, 1.0f);
        GizmosDraw(&gizmosState, camera);
      EndMode3D();

      GuiDraw(commandArray, uiState);
    EndDrawing();
  }
  GuiDeinit(uiState);
  CloseWindow();

  MemFree(manifolds);
  MemFree(physicsBodies);
  return 0;
}
