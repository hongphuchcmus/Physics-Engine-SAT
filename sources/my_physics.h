#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "raylib.h"
#include "raymath.h"
#include "geometry.h"
#include <string.h>
#include <stdint.h>
#include "rcamera.h"
#include <stdio.h>

#define GRAVITY 9.8f
#define COLLISION_TYPE_FACE_TO_VERTEX 0
#define COLLISION_TYPE_VERTEX_TO_FACE 1
#define COLLISION_TYPE_EDGE_TO_EDGE 2
#define COLLIDER_TYPE_STATIC 0
#define COLLIDER_TYPE_RIGID 1
#define COLLIDER_STATIC_MASS 1e20f

typedef struct Manifold {
  int32_t collisionType; 
  Vector3 seperationAxis;
  Vector3 contactPoints[8];
  Vector3 contactNormals[8];
  int32_t contactPointCount;
  float depth;
  bool overlapping;

  // Edge to edge debug
  Vector3 edge1Start;
  Vector3 edge1End;
  Vector3 edge2Start;
  Vector3 edge2End;
} Manifold;

typedef struct PhysicsBody {
  // Collision detection
  Vector3* vertices;
  int32_t vertexCount;
  int32_t* edges; // reference vertices, 2 entries form an edge
  int32_t edgeCount; // = size of edges/ 2 
  Vector3* edgeDirections;
  int32_t edgeDirectionCount;
  Vector3* faceNormals;
  int32_t faceNormalCount;
  // reference vertices and faceNormals, 2 entries form a face : (vertex on face, face normal)
  // Face normal index references will be offset by one
  // and be negative if the face is facing the opposite direction
  int32_t* faces;
  int32_t faceCount; 

  // Physical attributes
  Vector3 velocity;
  Quaternion rotation;
  Vector3 angularVelocity;
  Vector3 position;
  float mass;
  BoundingBox localBoundingBox;
  int32_t colliderType;
  float inertia;
  
  char bodyName[100]; 
} PhysicsBody;

Vector3 GetCenterOfContactPoints(Manifold* colInfo);
PhysicsBody LoadPhysicsBodyFromMesh(Mesh mesh);
void UnloadPhysicsBody(PhysicsBody physicsBody);
Manifold CheckCollisionPhysicsBodies(const PhysicsBody* body1, const PhysicsBody* body2);
void DrawFaceNormals(PhysicsBody physicsBody);
void DrawFaceNormalCoords(PhysicsBody physicsBody, Camera camera);
void DrawPhysicsVertexIndices(PhysicsBody physicsBody, Camera camera);
void SetColliderType(PhysicsBody* physicsBody, int32_t colliderType);
void PhysicsBodyUpdate(PhysicsBody* physicsBody, float delta);
void PhysicsBodyAddImpulse(PhysicsBody* physicsBody, float delta, Vector3 impulse);
void ResolveCollisionPhysicsBodies(PhysicsBody* body1, PhysicsBody* body2, Manifold* colInfo, float delta);
void WakeUp(PhysicsBody* physicsBody);
void ManifoldToString(Manifold* colInfo, char* buffer);
BoundingBox PhysicsBodyGetWorldBoundingBox(PhysicsBody* physicsBody);
BoundingBox PhysicsBodyGetWorldBoundingBoxMargin(PhysicsBody* physicsBody, float margin);
float InvInertia(PhysicsBody* physicsBody);
float InvMass(PhysicsBody* physicsBody);
float PhysicsBodyToString(const PhysicsBody* physicsBody, char* buffer);

#endif