#include "my_physics.h"

float physicsTimeScale = 1.0f;

// Find the distance between two edges, return the interaction on the first line
float f_LineToLineDistanceSqr(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, Vector3 *outR1)
{
  // Segment 1 Direction (Non Unit)
  Vector3 e1 = Vector3Subtract(q1, p1);
  // Segment 2 Direction (Non Unit)
  Vector3 e2 = Vector3Subtract(q2, p2);
  
  // Any points on these two segements 
  // r1 = p1 + t1 * e1  0<=t1<=1
  // r2 = p2 + t2 * e2  0<=t1<=1
  
  // The line connect the two segments
  Vector3 n = Vector3CrossProduct(e1, e2);

  // If two segments nearly parrallel,
  // return distance from one point on segment 1 to segment 2
  if (Vector3LengthSqr(n) < 0.0001f){
    // Formula:
    // d^2 = ((p2-p1) x e1).((p2-p1) x e1) / (e2.e2)
    
    // For the intersection, let choose the middle point of the shorert segment
    if (Vector3LengthSqr(e1) < Vector3LengthSqr(e2)){
      *outR1 = Vector3Add(p1, Vector3Scale(e1, 0.5f));
    } else {
      *outR1 = Vector3Add(p2, Vector3Scale(e2, 0.5f));
    }
    float dSqr = Vector3LengthSqr(Vector3CrossProduct(Vector3Subtract(p2, p1), e1)) / Vector3LengthSqr(e1);
    return dSqr;
  }
  
  // Find the closest point to the other line on each segment
  // t1 = (e2 x n).(p2-p1) / (n.n)
  // t2 = (e1 x n).(p2-p1) / (n.n)
  float t1 = Vector3DotProduct(Vector3CrossProduct(e2, n), Vector3Subtract(p2, p1)) / Vector3LengthSqr(n);
  float t2 = Vector3DotProduct(Vector3CrossProduct(e1, n), Vector3Subtract(p2, p1)) / Vector3LengthSqr(n);
  
  // Clamp to segment range [0,1]
  t1 = Clamp(t1, 0.0f, 1.0f);
  t2 = Clamp(t2, 0.0f, 1.0f);
  
  Vector3 r1 = Vector3Add(p1, Vector3Scale(e1, t1));
  Vector3 r2 = Vector3Add(p2, Vector3Scale(e2, t2));
  *outR1 = r1;
  
  return Vector3LengthSqr(Vector3Subtract(r1, r2));
}

// Find the face that align nearest to the seperation axis
void f_FindNearestFace(const PhysicsBody *body, Vector3 axis, Vector3 *outPoint, Vector3 *outNormal)
{
  float maxDot = -1.0f;
  float epsilon = 1e-4f;
  for (int32_t i = 0; i < 2 * body->faceCount; i += 2)
  {
    Vector3 point = body->vertices[body->faces[i]];
    Vector3 normal = {0};
    if (body->faces[i + 1] > 0)
    {
      normal = body->faceNormals[body->faces[i + 1] - 1];
    }
    else
    {
      normal = Vector3Negate(body->faceNormals[-body->faces[i + 1] - 1]);
    }
    point = Vector3Add(Vector3RotateByQuaternion(point, body->rotation), body->position);
    normal = Vector3RotateByQuaternion(normal, body->rotation);
    float dot = Vector3DotProduct(normal, axis);
    if (dot > maxDot)
    {
      maxDot = dot;
      *outPoint = point;
      *outNormal = normal;
    }
  }
}

float f_DistanceToPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal)
{
  float planeD = -Vector3DotProduct(planeNormal, planePoint);
  float d = (Vector3DotProduct(planeNormal, point) + planeD) /
            Vector3DotProduct(planeNormal, planeNormal);
  return fabs(d);
}

float f_CheckOverlapForAxis(const PhysicsBody *body1, const PhysicsBody *body2, Vector3 axis)
{
  float minBody1Comp = 10000.0f;
  int32_t minBody1Vertex = 0;
  float maxBody1Comp = -10000.0f;
  int32_t maxBody1Vertex = 0;
  for (int i = 0; i < body1->vertexCount; i++)
  {
    Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body1->vertices[i], body1->rotation), body1->position);
    float comp = Vector3DotProduct(vertex, axis);
    if (comp < minBody1Comp)
    {
      minBody1Comp = comp;
      minBody1Vertex = i;
    }
    if (comp > maxBody1Comp)
    {
      maxBody1Comp = comp;
      maxBody1Vertex = i;
    }
  }

  float minBody2Comp = 10000.0f;
  int32_t minBody2Vertex = 0;
  float maxBody2Comp = -10000.0f;
  int32_t maxBody2Vertex = 0;

  for (int i = 0; i < body2->vertexCount; i++)
  {
    Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body2->vertices[i], body2->rotation), body2->position);
    float comp = Vector3DotProduct(vertex, axis);
    if (comp < minBody2Comp)
    {
      minBody2Comp = comp;
      minBody2Vertex = i;
    }
    if (comp > maxBody2Comp)
    {
      maxBody2Comp = comp;
      maxBody2Vertex = i;
    }
  }
  float maxOfMin = minBody1Comp > minBody2Comp ? minBody1Comp : minBody2Comp;
  float minOfMax = maxBody1Comp < maxBody2Comp ? maxBody1Comp : maxBody2Comp;
  if (maxOfMin > minOfMax)
  {
    return -1;
  }
  return minOfMax - maxOfMin;
}

Vector3 GetCenterOfContactPoints(Manifold *colInfo)
{
  Vector3 res = {0};
  for (int32_t i = 0; i < colInfo->contactPointCount; i++)
  {
    res = Vector3Add(res, colInfo->contactPoints[i]);
  }
  if (colInfo->contactPointCount > 0)
  {
    return Vector3Scale(res, 1.0f / colInfo->contactPointCount);
  }
  return res;
}

PhysicsBody LoadPhysicsBodyFromMesh(Mesh mesh) {
  PhysicsBody physicsBody = {0};
  // Load vertices
  physicsBody.vertices = MemAlloc(mesh.vertexCount * sizeof(Vector3));
  physicsBody.vertexCount = 0;
  for (int32_t i = 0; i < mesh.vertexCount; i++) {
    Vector3 vertex = {mesh.vertices[i * 3], mesh.vertices[i * 3 + 1], mesh.vertices[i * 3 + 2]};
    bool duplicatedVertex = false;
    for (int32_t j = 0; j < physicsBody.vertexCount; j++) {
      if (Vector3Equals(vertex, physicsBody.vertices[j])) {
        duplicatedVertex = true;
        break;
      }
    }
    if (duplicatedVertex)
      continue;
    physicsBody.vertices[physicsBody.vertexCount++] = vertex;
    // Construct AABB
    if (vertex.x > physicsBody.localBoundingBox.max.x)
      physicsBody.localBoundingBox.max.x = vertex.x;
    if (vertex.y > physicsBody.localBoundingBox.max.y)
      physicsBody.localBoundingBox.max.y = vertex.y;
    if (vertex.z > physicsBody.localBoundingBox.max.z)
      physicsBody.localBoundingBox.max.z = vertex.z;

    if (vertex.x < physicsBody.localBoundingBox.min.x)
      physicsBody.localBoundingBox.min.x = vertex.x;
    if (vertex.y < physicsBody.localBoundingBox.min.y)
      physicsBody.localBoundingBox.min.y = vertex.y;
    if (vertex.z < physicsBody.localBoundingBox.min.z)
      physicsBody.localBoundingBox.min.z = vertex.z;
  }
  if (physicsBody.vertexCount != mesh.vertexCount) {
    physicsBody.vertices = MemRealloc(physicsBody.vertices, sizeof(Vector3) * physicsBody.vertexCount);
  }

  // Load edges and edge directions
  // Since this litterally extract edges from each triangle
  // a quad may have an edge in the middle, and an edge to edge collision
  // can happen from that middle of the face
  physicsBody.edges = MemAlloc(2 * 3 * mesh.triangleCount * sizeof(int32_t));
  physicsBody.edgeCount = 0;
  physicsBody.edgeDirections = MemAlloc(3 * mesh.triangleCount * sizeof(Vector3));
  physicsBody.edgeDirectionCount = 0;
  for (int32_t i = 0; i < mesh.triangleCount; i++) {
    Vector3 v1;
    Vector3 v2;
    Vector3 v3;
    if (mesh.indices != NULL){
      v1 = (Vector3) {mesh.vertices[mesh.indices[i*3] * 3], mesh.vertices[mesh.indices[i*3] * 3 + 1], mesh.vertices[mesh.indices[i*3] * 3 + 2]};
      v2 = (Vector3) {mesh.vertices[mesh.indices[i*3+1] * 3], mesh.vertices[mesh.indices[i*3+1] * 3 + 1], mesh.vertices[mesh.indices[i*3+1] * 3 + 2]};
      v3 = (Vector3) {mesh.vertices[mesh.indices[i*3+2] * 3], mesh.vertices[mesh.indices[i*3+2] * 3 + 1], mesh.vertices[mesh.indices[i*3+2] * 3 + 2]};
    } else {
      v1 = (Vector3) {mesh.vertices[i*9], mesh.vertices[i*9 + 1], mesh.vertices[i*9 + 2]};
      v2 = (Vector3) {mesh.vertices[i*9 + 3], mesh.vertices[i*9 + 4], mesh.vertices[i*9 + 5]};
      v3 = (Vector3) {mesh.vertices[i*9 + 6], mesh.vertices[i*9 + 7], mesh.vertices[i*9 + 8]};
    }

    Vector3 v1v2 = Vector3Normalize(Vector3Subtract(v2, v1));
    Vector3 v2v3 = Vector3Normalize(Vector3Subtract(v3, v2));
    Vector3 v3v1 = Vector3Normalize(Vector3Subtract(v3, v1));

    if (Vector3LengthSqr(v1v2) < 1e-4f || Vector3LengthSqr(v2v3) < 1e-4f || Vector3LengthSqr(v3v1) < 1e-4f) {
      continue;
    }

    bool dupDir1 = false; 
    bool dupDir2 = false;
    bool dupDir3 = false;

    for (int32_t j = 0; j < physicsBody.edgeDirectionCount; j++) {
      if (!dupDir1 && (Vector3LengthSqr(Vector3Subtract(physicsBody.edgeDirections[j], v1v2)) < 1e-4f || Vector3LengthSqr(Vector3Subtract(Vector3Negate(physicsBody.edgeDirections[j]), v1v2)) < 1e-4f)) {
        dupDir1 = true;
      }
      if (!dupDir2 && (Vector3LengthSqr(Vector3Subtract(physicsBody.edgeDirections[j], v2v3)) < 1e-4f || Vector3LengthSqr(Vector3Subtract(Vector3Negate(physicsBody.edgeDirections[j]), v2v3)) < 1e-4f)) {
        dupDir2 = true;
      }
      if (!dupDir3 && (Vector3LengthSqr(Vector3Subtract(physicsBody.edgeDirections[j], v3v1)) < 1e-4f || Vector3LengthSqr(Vector3Subtract(Vector3Negate(physicsBody.edgeDirections[j]), v3v1)) < 1e-4f)) {
        dupDir3 = true;
      }
    }

    int32_t v1Idx = -1;
    int32_t v2Idx = -1;
    int32_t v3Idx = -1;
    bool dupEdg1 = false;
    bool dupEdg2 = false;
    bool dupEdg3 = false;

    for (int32_t j = 0; j < physicsBody.vertexCount; j++) {
      if (v1Idx < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v1)) < 1e-8f) {
        v1Idx = j;
      }
      if (v2Idx < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v2)) < 1e-8f) {
        v2Idx = j;
      }
      if (v3Idx < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v3)) < 1e-8f) {
        v3Idx = j;
      }
    }

    for (int32_t j = 0; j < 2 * physicsBody.edgeCount; j += 2) {
      if (!dupEdg1 && ((v1Idx == physicsBody.edges[j] && v2Idx == physicsBody.edges[j + 1]) || (v2Idx == physicsBody.edges[j] && v1Idx == physicsBody.edges[j + 1]))) {
        dupEdg1 = true;
      }
      if (!dupEdg2 && ((v2Idx == physicsBody.edges[j] && v3Idx == physicsBody.edges[j + 1]) || (v3Idx == physicsBody.edges[j] && v2Idx == physicsBody.edges[j + 1]))) {
        dupEdg2 = true;
      }
      if (!dupEdg3 && ((v3Idx == physicsBody.edges[j] && v1Idx == physicsBody.edges[j + 1]) || (v1Idx == physicsBody.edges[j] && v3Idx == physicsBody.edges[j + 1]))) {
        dupEdg3 = true;
      }
    }
    if (!dupEdg1) {
      physicsBody.edges[2 * physicsBody.edgeCount] = v1Idx;
      physicsBody.edges[2 * physicsBody.edgeCount + 1] = v2Idx;
      physicsBody.edgeCount++;
    }
    if (!dupEdg2) {
      physicsBody.edges[2 * physicsBody.edgeCount] = v2Idx;
      physicsBody.edges[2 * physicsBody.edgeCount + 1] = v3Idx;
      physicsBody.edgeCount++;
    }
    if (!dupEdg3) {
      physicsBody.edges[2 * physicsBody.edgeCount] = v3Idx;
      physicsBody.edges[2 * physicsBody.edgeCount + 1] = v1Idx;
      physicsBody.edgeCount++;
    }

    if (!dupDir1)
      physicsBody.edgeDirections[physicsBody.edgeDirectionCount++] = v1v2;
    if (!dupDir2)
      physicsBody.edgeDirections[physicsBody.edgeDirectionCount++] = v2v3;
    if (!dupDir3)
      physicsBody.edgeDirections[physicsBody.edgeDirectionCount++] = v3v1;
  }
  if (physicsBody.edgeDirectionCount != 3 * mesh.triangleCount) {
    physicsBody.edgeDirections = MemRealloc(physicsBody.edgeDirections, physicsBody.edgeDirectionCount * sizeof(Vector3));
  }
  if (physicsBody.edgeCount != 3 * mesh.triangleCount) {
    physicsBody.edges = MemRealloc(physicsBody.edges, 2 * physicsBody.edgeCount * sizeof(int32_t));
  }
  // Load face normals
  physicsBody.faces = MemAlloc(2 * mesh.triangleCount * sizeof(int32_t));
  physicsBody.faceCount = 0;
  physicsBody.faceNormals = MemAlloc(mesh.triangleCount * sizeof(Vector3));
  physicsBody.faceNormalCount = 0;
  float epsilon = 0.0001f;
  for (int32_t i = 0; i < mesh.triangleCount; i++) {
    Vector3 v1;
    Vector3 v2;
    Vector3 v3;
    if (mesh.indices != NULL){
      v1 = (Vector3) {mesh.vertices[mesh.indices[i*3] * 3], mesh.vertices[mesh.indices[i*3] * 3 + 1], mesh.vertices[mesh.indices[i*3] * 3 + 2]};
      v2 = (Vector3) {mesh.vertices[mesh.indices[i*3+1] * 3], mesh.vertices[mesh.indices[i*3+1] * 3 + 1], mesh.vertices[mesh.indices[i*3+1] * 3 + 2]};
      v3 = (Vector3) {mesh.vertices[mesh.indices[i*3+2] * 3], mesh.vertices[mesh.indices[i*3+2] * 3 + 1], mesh.vertices[mesh.indices[i*3+2] * 3 + 2]};
    } else {
      v1 = (Vector3) {mesh.vertices[i*9], mesh.vertices[i*9 + 1], mesh.vertices[i*9 + 2]};
      v2 = (Vector3) {mesh.vertices[i*9 + 3], mesh.vertices[i*9 + 4], mesh.vertices[i*9 + 5]};
      v3 = (Vector3) {mesh.vertices[i*9 + 6], mesh.vertices[i*9 + 7], mesh.vertices[i*9 + 8]};
    }

    Vector3 meshNormal = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(v2, v1), Vector3Subtract(v3, v1)));
    // This would happen somehow 
    if (Vector3LengthSqr(meshNormal) < 1e-4f){
      continue;
    }
    
    for (int32_t j = 0; j < physicsBody.vertexCount; j++) {
      if (Vector3Equals(v1, physicsBody.vertices[j])) {
        physicsBody.faces[2 * physicsBody.faceCount] = j;
        break;
      }
    }

    bool duplicatedAxis = false;
    for (int32_t j = 0; j < physicsBody.faceNormalCount; j++) {
      float dot = Vector3DotProduct(meshNormal, physicsBody.faceNormals[j]);
      if (fabs(fabs(dot) - 1.0f) < epsilon) {
        duplicatedAxis = true;
        if (dot > 0.0f) {
          physicsBody.faces[2 * physicsBody.faceCount + 1] = j + 1;
        } else {
          physicsBody.faces[2 * physicsBody.faceCount + 1] = -(j + 1);
        }
        break;
      }
    }
    if (duplicatedAxis) {
      physicsBody.faceCount++;
      continue;
    }
    physicsBody.faceNormals[physicsBody.faceNormalCount++] = meshNormal;
    physicsBody.faces[2 * physicsBody.faceCount + 1] = physicsBody.faceNormalCount;  // -1 + 1 = 0
    physicsBody.faceCount++;
  }
  if (physicsBody.faceNormalCount != mesh.triangleCount) {
    physicsBody.faceNormals = MemRealloc(physicsBody.faceNormals, physicsBody.faceNormalCount * sizeof(Vector3));
  }
  if (physicsBody.faceCount != mesh.triangleCount) {
    physicsBody.faces = MemRealloc(physicsBody.faces, 2 * physicsBody.faceCount * sizeof(int32_t));
  }

  physicsBody.rotation = QuaternionIdentity();
  physicsBody.mass = 1.0f;
  physicsBody.inertia = 0.05f;
  physicsBody.colliderType = COLLIDER_TYPE_RIGID;

  return physicsBody;
}

void UnloadPhysicsBody(PhysicsBody physicsBody)
{
  MemFree(physicsBody.vertices);
  MemFree(physicsBody.faceNormals);
  MemFree(physicsBody.edgeDirections);
  MemFree(physicsBody.edges);
  MemFree(physicsBody.faces);
}

Manifold CheckCollisionPhysicsBodies(const PhysicsBody *body1, const PhysicsBody *body2)
{
  Manifold colInfo = {0};
  // Check bounding boxes overlap
  if (!CheckCollisionBoxes(PhysicsBodyGetWorldBoundingBox(body1), PhysicsBodyGetWorldBoundingBox(body2)))
  {
    return colInfo;
  }

  Matrix body1Matrix = QuaternionToMatrix(body1->rotation);
  Matrix body2Matrix = QuaternionToMatrix(body2->rotation);

  float minOverlapDepth = 1e30f;
  Vector3 minOverlapAxis = {0};
  int32_t minOverlapCollisionType = COLLISION_TYPE_VERTEX_TO_FACE;

  // Check with body1's normals
  for (int32_t i = 0; i < body1->faceNormalCount; i++)
  {
    if (IsKeyPressed(KEY_F1)){
      TraceLog(LOG_INFO, "face normal 1: %.2f, %.2f, %.2f", body1->faceNormals[i].x, body1->faceNormals[i].y, body1->faceNormals[i].z);
    }
    Vector3 axis = Vector3RotateByQuaternion(body1->faceNormals[i], body1->rotation);
    float depth = f_CheckOverlapForAxis(body1, body2, axis);
    if (depth < 0)
    {
      colInfo.seperationAxis = axis;
      return colInfo;
    }
    if (depth < minOverlapDepth)
    {
      minOverlapDepth = depth;
      minOverlapAxis = axis;
      minOverlapCollisionType = COLLISION_TYPE_FACE_TO_VERTEX;
    }
  }
  // Check with body2's normals
  for (int32_t i = 0; i < body2->faceNormalCount; i++)
  {
    if (IsKeyPressed(KEY_F1)){
      TraceLog(LOG_INFO, "face normal 2: %.2f, %.2f, %.2f", body2->faceNormals[i].x, body2->faceNormals[i].y, body2->faceNormals[i].z);
    }
    Vector3 axis = Vector3RotateByQuaternion(body2->faceNormals[i], body2->rotation);
    float depth = f_CheckOverlapForAxis(body1, body2, axis);
    if (depth < 0)
    {
      colInfo.seperationAxis = axis;
      return colInfo;
    }
    if (depth < minOverlapDepth)
    {
      minOverlapDepth = depth;
      minOverlapAxis = axis;
      minOverlapCollisionType = COLLISION_TYPE_VERTEX_TO_FACE;
    }
  }
  float epsilon = 0.01f;
  // Check with body1 & body2 crossed-product edge
  for (int32_t i = 0; i < body1->edgeDirectionCount; i++)
  {
    Vector3 body1Edge = Vector3RotateByQuaternion(body1->edgeDirections[i], body1->rotation);
    for (int32_t j = 0; j < body2->edgeDirectionCount; j++)
    {
      Vector3 body2Edge = Vector3RotateByQuaternion(body2->edgeDirections[j], body2->rotation);

      Vector3 axis = Vector3CrossProduct(body1Edge, body2Edge);
      if (Vector3LengthSqr(axis) <= epsilon * epsilon)
      {
        continue;
      }
      axis = Vector3Normalize(axis);
      float depth = f_CheckOverlapForAxis(body1, body2, axis);
      if (depth < 0)
      {
        colInfo.seperationAxis = axis;
        return colInfo;
      }
      if (depth < minOverlapDepth)
      {
        minOverlapDepth = depth;
        minOverlapAxis = axis;
        minOverlapCollisionType = COLLISION_TYPE_EDGE_TO_EDGE;
      }
    }
  }
  colInfo.overlapping = true;
  colInfo.seperationAxis = Vector3Normalize(minOverlapAxis);
  colInfo.depth = minOverlapDepth;
  colInfo.collisionType = minOverlapCollisionType;
  return colInfo;
}

void DrawFaceNormals(PhysicsBody physicsBody)
{
  for (int32_t i = 0; i < physicsBody.faceNormalCount; i++)
  {
    Vector3 normal = Vector3RotateByQuaternion(physicsBody.faceNormals[i], physicsBody.rotation);
    Vector3 drawPos = physicsBody.position;
    //Draw axis
    DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(normal), 1000.0f), drawPos), Vector3Add(Vector3Scale(normal, 1000.0f), drawPos), BLUE);
  }
}

void DrawFaceNormalCoords(PhysicsBody physicsBody, Camera camera)
{
  Vector3 camForward = GetCameraForward(&camera);
  for (int i = 0; i < physicsBody.faceNormalCount; i++)
  {
    Vector3 drawPos = Vector3Add(physicsBody.position, Vector3Scale(physicsBody.faceNormals[i], 2.0f));
    if (Vector3DotProduct(camForward, Vector3Subtract(drawPos, camera.position)) > 0)
    {
      Vector2 screenPos = GetWorldToScreen(drawPos, camera);
      DrawText(TextFormat("(%.1f, %.1f, %.1f)", physicsBody.faceNormals[i].x, physicsBody.faceNormals[i].y, physicsBody.faceNormals[i].z), (int)screenPos.x + 10, (int)screenPos.y - 4, 4, BLACK);
    }
  }
}

void DrawPhysicsVertexIndices(PhysicsBody physicsBody, Camera camera)
{
  Vector3 camForward = GetCameraForward(&camera);
  for (int32_t i = 0; i < physicsBody.vertexCount; i++)
  {
    Vector3 drawPos = Vector3Add(physicsBody.position, physicsBody.vertices[i]);
    Vector2 screenPos = GetWorldToScreen(drawPos, camera);
    DrawText(TextFormat("%d", i), (int)screenPos.x + 10, (int)screenPos.y - 4, 8, BLACK);
  }
}

void SetColliderType(PhysicsBody *physicsBody, int32_t colliderType)
{
  if (colliderType == COLLIDER_TYPE_STATIC)
  {
    physicsBody->colliderType = COLLIDER_TYPE_STATIC;
    physicsBody->mass = COLLIDER_STATIC_MASS;
  }
  else if (colliderType == COLLIDER_TYPE_RIGID)
  {
    physicsBody->colliderType = COLLIDER_TYPE_RIGID;
  }
}

void PhysicsBodyUpdate(PhysicsBody *physicsBody, float delta)
{
  delta *= physicsTimeScale;

  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC)
  {
    physicsBody->angularVelocity = (Vector3){0};
    physicsBody->velocity = (Vector3){0};
    return;
  }

  // Note to self: Don't ever use Vector3Equals() for stopping threshold, e.g Vector3Equals(linearMovement, Vector3Zero())
  // It has a very small epsilon which is more suitable for removing vertex duplicates
  float epsilon = 0.0001f;

  float angularDamping = 2.0f;
  physicsBody->angularVelocity = Vector3Scale(physicsBody->angularVelocity, Clamp(1 - angularDamping * delta, 0, 1));

  Vector3 angularMovement = Vector3Scale(physicsBody->angularVelocity, delta);
  if (Vector3LengthSqr(angularMovement) > epsilon * epsilon)
  {
    physicsBody->rotation = QuaternionMultiply(QuaternionFromEuler(angularMovement.x, angularMovement.y, angularMovement.z), physicsBody->rotation);
  }

  float linearDamping = 1.0f;
  float vy = physicsBody->velocity.y - GRAVITY * delta;
  physicsBody->velocity = Vector3Scale(physicsBody->velocity, Clamp(1 - linearDamping * delta, 0, 1));
  physicsBody->velocity.y = vy;

  Vector3 linearMovement = Vector3Scale(physicsBody->velocity, delta);
  if (Vector3LengthSqr(linearMovement) > epsilon * epsilon)
  {
    physicsBody->position = Vector3Add(physicsBody->position, linearMovement);
  }
}

void PhysicsBodyAddImpulse(PhysicsBody *physicsBody, float delta, Vector3 impulse)
{
  physicsBody->velocity = Vector3Add(physicsBody->velocity, impulse);
}

void ResolveCollisionPhysicsBodies(PhysicsBody *body1, PhysicsBody *body2, Manifold *manifold, float delta)
{
  if (!manifold->overlapping)
  {
    return;
  }

  // Seperation axis but make it in the direction from body1 -> body2
  Vector3 body1ToBody2SepAxis = Vector3DotProduct(Vector3Subtract(body2->position, body1->position), manifold->seperationAxis) < 0 ? Vector3Negate(manifold->seperationAxis) : manifold->seperationAxis;
  // Seperation axis but make it in the direction from body2 -> body1
  Vector3 body2ToBody1SepAxis = Vector3Negate(body1ToBody2SepAxis);
  float colMargin = 0.0f; // An extra margin to prevent objects from sticking together again

  // Seperating two objects
  if (body1->colliderType == COLLIDER_TYPE_RIGID && body2->colliderType == COLLIDER_TYPE_RIGID)
  {
    body1->position = Vector3Add(body1->position, Vector3Scale(body2ToBody1SepAxis, manifold->depth * 0.5f + colMargin));
    body2->position = Vector3Add(body2->position, Vector3Scale(body1ToBody2SepAxis, manifold->depth * 0.5f + colMargin));
  }
  else if (body1->colliderType == COLLIDER_TYPE_RIGID && body2->colliderType == COLLIDER_TYPE_STATIC)
  {
    body1->position = Vector3Add(body1->position, Vector3Scale(body2ToBody1SepAxis, manifold->depth + colMargin));
  }
  else
  {
    body2->position = Vector3Add(body2->position, Vector3Scale(body1ToBody2SepAxis, manifold->depth + colMargin));
  }

  if (IsKeyPressed(KEY_F1)){
    TraceLog(LOG_INFO, "debugging");
  }

  // Find the contact point
  switch (manifold->collisionType)
  {
  case COLLISION_TYPE_FACE_TO_VERTEX:
  {
    Vector3 faceRefPoint = {0};
    Vector3 faceRefNormal = {0};
    // Reference face
    f_FindNearestFace(body1, body1ToBody2SepAxis, &faceRefPoint, &faceRefNormal);

    float *vertexDistances = MemAlloc(body2->vertexCount * sizeof(float));

    float minDistance = 1e30f;
    for (int32_t i = 0; i < body2->vertexCount; i++)
    {
      Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body2->vertices[i], body2->rotation), body2->position);
      float distance = f_DistanceToPlane(vertex, faceRefPoint, faceRefNormal);
      vertexDistances[i] = distance;
      if (distance < minDistance)
      {
        minDistance = distance;
      }
      if (IsKeyPressed(KEY_F1))
      {
        TraceLog(LOG_INFO, "sep axis: %.2f, %.2f, %.2f", manifold->seperationAxis.x, manifold->seperationAxis.y, manifold->seperationAxis.z);
        TraceLog(LOG_INFO, "ref point: %.2f, %.2f, %.2f", faceRefPoint.x, faceRefPoint.y, faceRefPoint.z);
        TraceLog(LOG_INFO, "ref normal: %.2f, %.2f, %.2f", faceRefNormal.x, faceRefNormal.y, faceRefNormal.z);
        TraceLog(LOG_INFO, "dist 2 plane: %.2f", distance);
      }
    }
    manifold->contactPointCount = 0;
    // Add more vertices with approximately the same distance
    for (int32_t i = 0; i < body2->vertexCount; i++)
    {
      if (fabs(vertexDistances[i] - minDistance) <= 0.05f)
      {
        Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body2->vertices[i], body2->rotation), body2->position);
        manifold->contactPoints[manifold->contactPointCount] = vertex;
        manifold->contactNormals[manifold->contactPointCount] = faceRefNormal;
        manifold->contactPointCount++;
      }
    }


    MemFree(vertexDistances);
    break;
  }
  case COLLISION_TYPE_VERTEX_TO_FACE:
  {
    Vector3 faceRefPoint = {0};
    Vector3 faceRefNormal = {0};
    // Reference face
    f_FindNearestFace(body2, body2ToBody1SepAxis, &faceRefPoint, &faceRefNormal);

    float *vertexDistances = MemAlloc(body1->vertexCount * sizeof(float));

    float minDistance = 1e30f;
    for (int32_t i = 0; i < body1->vertexCount; i++)
    {
      Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body1->vertices[i], body2->rotation), body1->position);
      float distance = f_DistanceToPlane(vertex, faceRefPoint, faceRefNormal);
      vertexDistances[i] = distance;
      if (distance < minDistance)
      {
        minDistance = distance;
      }
    }

    // Add more vertices with approximately the same distance
    manifold->contactPointCount = 0;
    for (int32_t i = 0; i < body1->vertexCount; i++)
    {
      if (fabs(vertexDistances[i] - minDistance) <= 0.05f)
      {
        Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body1->vertices[i], body2->rotation), body1->position);
        manifold->contactPoints[manifold->contactPointCount++] = vertex;
        manifold->contactNormals[manifold->contactPointCount] = faceRefNormal;
      }
    }

    MemFree(vertexDistances);
    break;
  }
  case COLLISION_TYPE_EDGE_TO_EDGE:
  {
    // Find the candidates for the colliding edge of body1
    // Choose from the edges that pendicular to the seperation axis
    int32_t body1EdgeCandidatesCount = 0;
    int32_t *body1EdgeCandidates = MemAlloc(2 * body1->edgeCount * sizeof(int32_t));

    for (int32_t i = 0; i < 2 * body1->edgeCount; i += 2)
    {
      Vector3 v1 = Vector3RotateByQuaternion(body1->vertices[body1->edges[i]], body1->rotation);
      Vector3 v2 = Vector3RotateByQuaternion(body1->vertices[body1->edges[i + 1]], body1->rotation);
      Vector3 edgeDir = Vector3Subtract(v2, v1);
      if (fabs(Vector3DotProduct(edgeDir, manifold->seperationAxis)) < 0.01f)
      {
        body1EdgeCandidates[2 * body1EdgeCandidatesCount] = body1->edges[i];
        body1EdgeCandidates[2 * body1EdgeCandidatesCount + 1] = body1->edges[i + 1];
        body1EdgeCandidatesCount++;
      }
    }
    // Find the candidates for the colliding edge of body2
    // Choose from the edges that pendicular to the seperation axis
    int32_t body2EdgeCandidatesCount = 0;
    int32_t *body2EdgeCandidates = MemAlloc(2 * body2->edgeCount * sizeof(int32_t));
    for (int32_t i = 0; i < 2 * body2->edgeCount; i += 2)
    {
      Vector3 v1 = Vector3RotateByQuaternion(body2->vertices[body2->edges[i]], body2->rotation);
      Vector3 v2 = Vector3RotateByQuaternion(body2->vertices[body2->edges[i + 1]], body2->rotation);
      Vector3 edgeDir = Vector3Subtract(v2, v1);
      if (fabs(Vector3DotProduct(edgeDir, manifold->seperationAxis)) < 0.01f)
      {
        body2EdgeCandidates[2 * body2EdgeCandidatesCount] = body2->edges[i];
        body2EdgeCandidates[2 * body2EdgeCandidatesCount + 1] = body2->edges[i + 1];
        body2EdgeCandidatesCount++;
      }
    }
    // Find the closest edge pair
    float minDistanceSqr = 1e30f;
    Vector3 intersection = {0};
    Vector3 normal = {0};
    for (int32_t i = 0; i < 2 * body1EdgeCandidatesCount; i += 2)
    {
      Vector3 v1Start = Vector3Add(Vector3RotateByQuaternion(body1->vertices[body1EdgeCandidates[i]], body1->rotation), body1->position);
      Vector3 v1End = Vector3Add(Vector3RotateByQuaternion(body1->vertices[body1EdgeCandidates[i + 1]], body1->rotation), body1->position);
      for (int32_t j = 0; j < 2 * body2EdgeCandidatesCount; j += 2)
      {
        Vector3 v2Start = Vector3Add(Vector3RotateByQuaternion(body2->vertices[body2EdgeCandidates[j]], body2->rotation), body2->position);
        Vector3 v2End = Vector3Add(Vector3RotateByQuaternion(body2->vertices[body2EdgeCandidates[j + 1]], body2->rotation), body2->position);
        Vector3 intersectionOnV1 = {0};
        float distanceSqr = f_LineToLineDistanceSqr(v1Start, v1End, v2Start, v2End, &intersectionOnV1);
        if (distanceSqr < minDistanceSqr)
        {
          minDistanceSqr = distanceSqr;
          intersection = intersectionOnV1;
          normal = Vector3CrossProduct(Vector3Subtract(v1End, v1Start), Vector3Subtract(v2End, v2Start));
          if (Vector3LengthSqr(normal) < 1e-4f)
          {
            normal = body2ToBody1SepAxis;
          }
          manifold->edge1Start = v1Start;
          manifold->edge1End = v1End;
          manifold->edge2Start = v2Start;
          manifold->edge2End = v2End;
        }
      }
    }

    // Default direction should be from object 1 to object 2
    if (Vector3DotProduct(normal, Vector3Subtract(body2->position, body1->position)) < 0){
      normal = Vector3Negate(normal);
    }

    MemFree(body1EdgeCandidates);
    MemFree(body2EdgeCandidates);

    manifold->contactPointCount = 1;
    manifold->contactPoints[0] = intersection;
    manifold->contactNormals[0] = Vector3Normalize(normal);
  }
  default:
    break;
  }

  if (physicsTimeScale  < 1e-4f){
    return;
  }
  
  delta *= physicsTimeScale;
  // Calculate the impulse base on the velocity of center of mass and the angular velocity of each object
  // e : bounciness 
  float e = 0.1f;
  Vector3 totalImpulse = {0};
  // Vectors from center of masses to contact point
  for (int32_t i = 0; i < manifold->contactPointCount; i++)
  {
    Vector3 p = manifold->contactPoints[i]; 
    Vector3 n = manifold->contactNormals[i];
    Vector3 r1 = Vector3Subtract(p, body1->position);
    Vector3 r2 = Vector3Subtract(p, body2->position);
    // m_eff : Mass effective, approximated for 2 point masses
    float m_eff = InvMass(body1) + InvMass(body2) + Vector3LengthSqr(Vector3CrossProduct(r1, n)) * InvInertia(body1) + Vector3LengthSqr(Vector3CrossProduct(r2, n)) * InvInertia(body2);
    // vR : relative velocity between two point of each of object at the contact point
    Vector3 vR = Vector3Subtract(
      Vector3Add(body1->velocity, Vector3CrossProduct(body1->angularVelocity, r1)),
      Vector3Add(body2->velocity, Vector3CrossProduct(body2->angularVelocity, r2))
    );
    // Normal impulse
    Vector3 jN = Vector3Scale(n, -(1+e) * Vector3DotProduct(vR, n) / m_eff);

    // Tangential impulse
    Vector3 vN = Vector3Project(vR, n);
    Vector3 vT = Vector3Subtract(vR, vN);

    Vector3 jT = {0};
    // TODO: Coulomb limit: |jT| < mu.|jN|
    // where u : friction coeffiecnt, depend on physics materials
    if (Vector3LengthSqr(vT) >= 1e-8f){
      Vector3 t = Vector3Normalize(vT);
      // m_eff : Mass effective for tangential
      float m_effT = InvMass(body1) + InvMass(body2) + Vector3LengthSqr(Vector3CrossProduct(r1, t)) * InvInertia(body1) + Vector3LengthSqr(Vector3CrossProduct(r2, t)) * InvInertia(body2);
      jT = Vector3Scale(vT, -1.0f / m_effT);
    }

    totalImpulse = Vector3Add(totalImpulse, Vector3Add(jN, jT));
  }
  totalImpulse = Vector3Scale(totalImpulse, 1.0f / (float)manifold->contactPointCount);
  
  body1->velocity = Vector3Add(body1->velocity, Vector3Scale(totalImpulse, InvMass(body1)));
  body2->velocity = Vector3Add(body2->velocity, Vector3Scale(totalImpulse, -InvMass(body2)));

  Vector3 contactPointCenter = GetCenterOfContactPoints(manifold);

  // Apply torques
  // On body1
  Vector3 r1 = Vector3Subtract(contactPointCenter, body1->position); 
  Vector3 f1 = totalImpulse;
  Vector3 torque1 = Vector3CrossProduct(r1, f1);
  Vector3 angAccelDelta = Vector3Scale(torque1, 1.0f / body1->inertia * delta);
  body1->angularVelocity = Vector3Add(body1->angularVelocity, angAccelDelta);

  // On body2
  Vector3 r2 = Vector3Subtract(contactPointCenter, body2->position);
  Vector3 f2 = Vector3Negate(totalImpulse);
  Vector3 torque2 = Vector3CrossProduct(r2, f2);
  Vector3 angAccelDelta2 = Vector3Scale(torque2, 1.0f / body2->inertia * delta);
  body2->angularVelocity = Vector3Add(body2->angularVelocity, angAccelDelta2);
}

void WakeUp(PhysicsBody *physicsBody)
{
  // physicsBody->sleepingTimer = 0.0f;
  // physicsBody->isSleeping = false;
}

void ManifoldToString(Manifold *colInfo, char *buffer)
{
  char colType[100] = "null";
  if (colInfo->overlapping)
  {
    switch (colInfo->collisionType)
    {
    case COLLISION_TYPE_EDGE_TO_EDGE:
    {
      TextCopy(colType, "edge-to-edge");
      break;
    }
    case COLLISION_TYPE_FACE_TO_VERTEX:
    {
      TextCopy(colType, "face-to-vertex");
      break;
    }
    case COLLISION_TYPE_VERTEX_TO_FACE:
    {
      TextCopy(colType, "vertex-to-face");
      break;
    }
    default:
      break;
    }
  }
  
  sprintf(buffer, "Collision: %s\ncontacts: %d", colType, colInfo->contactPointCount);
}

BoundingBox PhysicsBodyGetWorldBoundingBox(PhysicsBody *physicsBody)
{
  BoundingBox bb = physicsBody->localBoundingBox;
  Vector3 points[8] = {
      bb.min,
      bb.max,
      (Vector3){bb.max.x, bb.min.y, bb.min.z},
      (Vector3){bb.min.x, bb.max.y, bb.min.z},
      (Vector3){bb.min.x, bb.min.y, bb.max.z},
      (Vector3){bb.min.x, bb.max.y, bb.max.z},
      (Vector3){bb.max.x, bb.min.y, bb.max.z},
      (Vector3){bb.max.x, bb.max.y, bb.min.z}};
  Vector3 newMin = (Vector3){1e30f, 1e30f, 1e30f};
  Vector3 newMax = (Vector3){-1e30f, -1e30f, -1e30f};
  for (int32_t i = 0; i < 8; i++)
  {
    points[i] = Vector3RotateByQuaternion(points[i], physicsBody->rotation);
    if (newMin.x > points[i].x)
      newMin.x = points[i].x;
    if (newMin.y > points[i].y)
      newMin.y = points[i].y;
    if (newMin.z > points[i].z)
      newMin.z = points[i].z;
    if (newMax.x < points[i].x)
      newMax.x = points[i].x;
    if (newMax.y < points[i].y)
      newMax.y = points[i].y;
    if (newMax.z < points[i].z)
      newMax.z = points[i].z;
  }
  newMin = Vector3Add(newMin, physicsBody->position);
  newMax = Vector3Add(newMax, physicsBody->position);
  return (BoundingBox){newMin, newMax};
}

BoundingBox PhysicsBodyGetWorldBoundingBoxMargin(PhysicsBody *physicsBody, float margin)
{
  BoundingBox bb = PhysicsBodyGetWorldBoundingBox(physicsBody);
  bb.min = Vector3Subtract(bb.min, (Vector3){margin, margin, margin});
  bb.max = Vector3Add(bb.max, (Vector3){margin, margin, margin});
  return bb;
}

float InvInertia(PhysicsBody *physicsBody) {
  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC){
    return 0.0f;
  }
  return 1.0f / physicsBody->mass;
}

float InvMass(PhysicsBody *physicsBody){
  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC){
    return 0.0f;
  }
  return physicsBody->inertia;
}

float PhysicsBodyToString(const PhysicsBody* physicsBody, char *buffer) {
  sprintf(buffer, "Body Name:%s\nv:(%.2f, %.2f, %.2f)\nav:(%.2f, %.2f, %.2f)", physicsBody->bodyName,
    physicsBody->velocity.x, physicsBody->velocity.y, physicsBody->velocity.z,
    physicsBody->angularVelocity.x, physicsBody->angularVelocity.y, physicsBody->angularVelocity.z);
}
