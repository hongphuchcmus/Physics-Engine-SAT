#include "my_physics.h"

float physicsTimeScale = 1.0f;

// Find the area of a convex polygon in 3D space
static float f_AreaConvexPoly(Vector3* vertices, int32_t vertexCount){
 if (vertexCount < 3){ 
  return 0;
 }
 // First we have to sort the vertices
 Vector3 center = {0};
 for (int32_t i = 0; i < vertexCount; i++){
  center = Vector3Add(center, vertices[i]);
 }
 center = Vector3Scale(center, 1.0f / vertexCount);
 Vector3 refNormal = Vector3CrossProduct(Vector3Subtract(vertices[0], center), Vector3Subtract(vertices[1], center));
 
 for (int32_t i = 0; i < vertexCount-1; i++)
 {
  bool swapped = false;
  for (int32_t j = 0; j < vertexCount-1-i; j++)
  {
    Vector3 a = vertices[i]; Vector3 b = vertices[j];
    Vector3 va = Vector3Subtract(a, center);
    Vector3 vb = Vector3Subtract(b, center);

    float dot = Vector3DotProduct(va, vb);
    Vector3 cross = Vector3CrossProduct(va, vb);
    float sign = Vector3DotProduct(cross, refNormal);
    if (sign > 0 || (fabs(sign) < 1e-4f && dot > 0)){
      Vector3 temp = vertices[i];
      vertices[i] = vertices[j];
      vertices[j] = temp;
      swapped = true;
    }
  }
  if (!swapped) break;
 }
 
 float totalArea = 0;
 for (int32_t i = 0; i < vertexCount -1; i++)
 {
  Vector3 edge1 = Vector3Subtract(vertices[i], center);
  Vector3 edge2 = Vector3Subtract(vertices[i+1], center);
  float area = 0.5f * Vector3Length(Vector3CrossProduct(edge1, edge2));
  totalArea += area;
 }
 return totalArea;
}

// Return the index of the face removed
static int f_RemoveTriangle(PhysicsBody *physicsBody, Triangle triangle) {
  Vector3 refPoint = triangle.p1;
  Vector3 refNormal = GetTriangleNormal(triangle);
  for (int32_t i = 0; i < physicsBody->faceCount; i++) {
    Vector3 v1 = physicsBody->vertices[physicsBody->faces[2 * i]];
    Vector3 normal = physicsBody->faceNormals[abs(physicsBody->faces[2 * i + 1]) - 1];
    if (physicsBody->faces[2 * i + 1] < 0) {
      normal = Vector3Negate(normal);
    }
    if (Vector3LengthSqr(Vector3Subtract(v1, refPoint)) < 1e-4f && Vector3LengthSqr(Vector3Subtract(normal, refNormal)) < 1e-4f) {
      physicsBody->faces[2 * i] = physicsBody->faces[2 * physicsBody->faceCount - 2];
      physicsBody->faces[2 * i + 1] = physicsBody->faces[2 * physicsBody->faceCount - 1];
      physicsBody->faceCount--;
      return i;
    }
  }
  return -1;
}

// Load 3 vertices of a triangle from a mesh
static Vector3 *f_GetTriangleVertices(const Mesh *mesh, int32_t triangleIndex, Vector3 *vertices) {
  if (mesh->indices != NULL) {
    for (int i = 0; i < 3; i++) {
      int idx = mesh->indices[triangleIndex * 3 + i] * 3;
      vertices[i] = (Vector3){
          mesh->vertices[idx],
          mesh->vertices[idx + 1],
          mesh->vertices[idx + 2]};
    }
  } else {
    for (int i = 0; i < 3; i++) {
      int idx = triangleIndex * 9 + i * 3;
      vertices[i] = (Vector3){
          mesh->vertices[idx],
          mesh->vertices[idx + 1],
          mesh->vertices[idx + 2]};
    }
  }
  return vertices;
}

static void f_RemoveEdge(Edge edge, PhysicsBody *physicsBody) {
  int32_t v1Idx = -1;
  int32_t v2Idx = -1;
  for (int32_t i = 0; i < physicsBody->vertexCount; i++) {
    if (v1Idx < 0 && Vector3Equals(physicsBody->vertices[i], edge.p1)) {
      v1Idx = i;
    }
    if (v2Idx < 0 && Vector3Equals(physicsBody->vertices[i], edge.p2)) {
      v2Idx = i;
    }
  }
  if (v1Idx < 0 || v2Idx < 0) {
    return;
  }
  for (int32_t i = 0; i < 2 * physicsBody->edgeCount; i += 2) {
    if ((physicsBody->edges[i] == v1Idx && physicsBody->edges[i + 1] == v2Idx) || (physicsBody->edges[i] == v2Idx && physicsBody->edges[i + 1] == v1Idx)) {
      physicsBody->edges[i] = physicsBody->edges[2 * physicsBody->edgeCount - 2];
      physicsBody->edges[i + 1] = physicsBody->edges[2 * physicsBody->edgeCount - 1];
      physicsBody->edgeCount--;
      break;
    }
  }
}

// Return the smallest depth and axis to the nearest edge
// Threshold is the minimum distance for the selection to happen
static float f_NearestEdgeAxis(const PhysicsBody *sphere, const PhysicsBody *convex, Vector3 *outAxis, Vector3 *overlapPoint) {
  float minDistanceSqr = 1e30f;
  Vector3 minToEdge = {0};
  for (int32_t i = 0; i < 2 * convex->edgeCount; i += 2) {
    //         p
    //        /|\     
    //       / | \    
    //      /  |  \   
    //    v1---o---v2
    //    <__t_>
    Vector3 v1 = Vector3Add(convex->position, Vector3RotateByQuaternion(convex->vertices[convex->edges[i]], convex->rotation));
    Vector3 v2 = Vector3Add(convex->position, Vector3RotateByQuaternion(convex->vertices[convex->edges[i + 1]], convex->rotation));
    Vector3 v1v2 = Vector3Subtract(v2, v1);
    Vector3 v1p = Vector3Subtract(sphere->position, v1);
    float v1v2MagSqr = Vector3LengthSqr(v1v2);
    float t = Vector3DotProduct(v1p, v1v2) / v1v2MagSqr;  // 0 <= t <= 1
    if (t < 0) {
      t = 0;
    } else if (t > 1) {
      t = 1;
    }
    Vector3 closestPoint = Vector3Add(v1, Vector3Scale(v1v2, t));
    Vector3 toEdge = Vector3Subtract(closestPoint, sphere->position);
    float distanceSqr = Vector3LengthSqr(toEdge);
    if (distanceSqr < minDistanceSqr) {
      minDistanceSqr = distanceSqr;
      minToEdge = toEdge;
    }
  }
  // No overlapping with this edge
  if (minDistanceSqr > sphere->radius * sphere->radius) {
    *outAxis = Vector3Normalize(minToEdge);
    *overlapPoint = Vector3Add(sphere->position, minToEdge);
    return -1;
  }
  float minToEdgeMag = sqrtf(minDistanceSqr);
  *outAxis = Vector3Scale(minToEdge, 1.0f / minToEdgeMag);
  *overlapPoint = Vector3Add(sphere->position, minToEdge);
  return sphere->radius - minToEdgeMag;
}

// Find the distance between two edges, return the interaction on the first line
static float f_LineToLineDistanceSqr(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, Vector3 *outR1) {
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
  if (Vector3LengthSqr(n) < 0.0001f) {
    // Formula:
    // d^2 = ((p2-p1) x e1).((p2-p1) x e1) / (e2.e2)

    // For the intersection, let choose the middle point of the shorert segment
    if (Vector3LengthSqr(e1) < Vector3LengthSqr(e2)) {
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
static void f_FindMostAlignedFace(const PhysicsBody *body, Vector3 axis, Vector3 *outPoint, Vector3 *outNormal) {
  float maxDot = -1.0f;
  float epsilon = 1e-4f;
  for (int32_t i = 0; i < 2 * body->faceCount; i += 2) {
    Vector3 point = body->vertices[body->faces[i]];
    Vector3 normal = {0};
    if (body->faces[i + 1] > 0) {
      normal = body->faceNormals[body->faces[i + 1] - 1];
    } else {
      normal = Vector3Negate(body->faceNormals[-body->faces[i + 1] - 1]);
    }
    point = Vector3Add(Vector3RotateByQuaternion(point, body->rotation), body->position);
    normal = Vector3RotateByQuaternion(normal, body->rotation);
    float dot = Vector3DotProduct(normal, axis);
    if (dot > maxDot) {
      maxDot = dot;
      *outPoint = point;
      *outNormal = normal;
    }
  }
}

static float f_DistanceToPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal) {
  float planeD = -Vector3DotProduct(planeNormal, planePoint);
  // planeNormal must be normalized
  float d = Vector3DotProduct(planeNormal, point) + planeD;
  return d;
}

static Vector3 f_ProjectOntoPlane(Vector3 point, Vector3 planePoint, Vector3 planeNormal) {
  float planeD = -Vector3DotProduct(planeNormal, planePoint);
  // planeNormal must be normalized
  float d = Vector3DotProduct(planeNormal, point) + planeD;
  return Vector3Subtract(point, Vector3Scale(planeNormal, d));
}

// Return the depth of penertration, return -1 if no overlapping occured
static float f_CheckConvexOverlap(const PhysicsBody *body1, const PhysicsBody *body2, Vector3 axis) {
  float minBody1Comp = 1e30f;
  int32_t minBody1Vertex = 0;
  float maxBody1Comp = -1e30f;
  int32_t maxBody1Vertex = 0;
  for (int i = 0; i < body1->vertexCount; i++) {
    Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body1->vertices[i], body1->rotation), body1->position);
    float comp = Vector3DotProduct(vertex, axis);
    if (comp < minBody1Comp) {
      minBody1Comp = comp;
      minBody1Vertex = i;
    }
    if (comp > maxBody1Comp) {
      maxBody1Comp = comp;
      maxBody1Vertex = i;
    }
  }

  float minBody2Comp = 1e30f;
  int32_t minBody2Vertex = 0;
  float maxBody2Comp = -1e30f;
  int32_t maxBody2Vertex = 0;

  for (int i = 0; i < body2->vertexCount; i++) {
    Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(body2->vertices[i], body2->rotation), body2->position);
    float comp = Vector3DotProduct(vertex, axis);
    if (comp < minBody2Comp) {
      minBody2Comp = comp;
      minBody2Vertex = i;
    }
    if (comp > maxBody2Comp) {
      maxBody2Comp = comp;
      maxBody2Vertex = i;
    }
  }
  float maxOfMin = minBody1Comp > minBody2Comp ? minBody1Comp : minBody2Comp;
  float minOfMax = maxBody1Comp < maxBody2Comp ? maxBody1Comp : maxBody2Comp;
  if (maxOfMin > minOfMax) {
    return -1;
  }
  return minOfMax - maxOfMin;
}

static float f_CheckSphereOverlap(const PhysicsBody *body1, const PhysicsBody *body2) {
  Vector3 axis = Vector3Normalize(Vector3Subtract(body2->position, body1->position));
  float minBody1Comp = Vector3DotProduct(axis, Vector3Add(body1->position, Vector3Scale(axis, -body1->radius)));
  float maxBody1Comp = Vector3DotProduct(axis, Vector3Add(body1->position, Vector3Scale(axis, body1->radius)));
  float minBody2Comp = Vector3DotProduct(axis, Vector3Add(body2->position, Vector3Scale(axis, -body2->radius)));
  float maxBody2Comp = Vector3DotProduct(axis, Vector3Add(body2->position, Vector3Scale(axis, body2->radius)));

  float maxOfMin = minBody1Comp > minBody2Comp ? minBody1Comp : minBody2Comp;
  float minOfMax = maxBody1Comp < maxBody2Comp ? maxBody1Comp : maxBody2Comp;
  if (maxOfMin > minOfMax) {
    return -1;
  }
  return minOfMax - maxOfMin;
}

static float f_CheckSphereConvexOverlap(const PhysicsBody *sphere, const PhysicsBody *convex, Vector3 axis) {
  // Make the axis point from sphere to convex
  if (Vector3DotProduct(Vector3Subtract(convex->position, sphere->position), axis) < 0) {
    axis = Vector3Negate(axis);
  }

  float minBody1Comp = Vector3DotProduct(axis, Vector3Add(sphere->position, Vector3Scale(axis, -sphere->radius)));
  float maxBody1Comp = Vector3DotProduct(axis, Vector3Add(sphere->position, Vector3Scale(axis, sphere->radius)));

  float minBody2Comp = 1e30f;
  int32_t minBody2Vertex = 0;
  float maxBody2Comp = -1e30f;
  int32_t maxBody2Vertex = 0;

  for (int i = 0; i < convex->vertexCount; i++) {
    Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(convex->vertices[i], convex->rotation), convex->position);
    float comp = Vector3DotProduct(vertex, axis);
    if (comp < minBody2Comp) {
      minBody2Comp = comp;
      minBody2Vertex = i;
    }
    if (comp > maxBody2Comp) {
      maxBody2Comp = comp;
      maxBody2Vertex = i;
    }
  }

  float maxOfMin = minBody1Comp > minBody2Comp ? minBody1Comp : minBody2Comp;
  float minOfMax = maxBody1Comp < maxBody2Comp ? maxBody1Comp : maxBody2Comp;
  if (maxOfMin > minOfMax) {
    return -1;
  }
  return minOfMax - maxOfMin;
}

Vector3 GetCenterOfContactPoints(Manifold *manifold) {
  Vector3 res = {0};
  for (int32_t i = 0; i < manifold->contactPointCount; i++) {
    res = Vector3Add(res, manifold->contactPoints[i]);
  }
  if (manifold->contactPointCount > 0) {
    return Vector3Scale(res, 1.0f / manifold->contactPointCount);
  }
  return res;
}

// This code is very messy, but it get the job done
PhysicsBody LoadPhysicsBodyFromConvexMesh(Mesh mesh) {
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
    if (vertex.x > physicsBody.boundingBox.max.x)
      physicsBody.boundingBox.max.x = vertex.x;
    if (vertex.y > physicsBody.boundingBox.max.y)
      physicsBody.boundingBox.max.y = vertex.y;
    if (vertex.z > physicsBody.boundingBox.max.z)
      physicsBody.boundingBox.max.z = vertex.z;

    if (vertex.x < physicsBody.boundingBox.min.x)
      physicsBody.boundingBox.min.x = vertex.x;
    if (vertex.y < physicsBody.boundingBox.min.y)
      physicsBody.boundingBox.min.y = vertex.y;
    if (vertex.z < physicsBody.boundingBox.min.z)
      physicsBody.boundingBox.min.z = vertex.z;
  }
  if (physicsBody.vertexCount != mesh.vertexCount) {
    physicsBody.vertices = MemRealloc(physicsBody.vertices, sizeof(Vector3) * physicsBody.vertexCount);
  }

  // Load face normals and edges at the same time
  Triangle *triangles = MemAlloc(mesh.triangleCount * sizeof(Triangle));
  int32_t triangleCount = 0;

  physicsBody.faces = MemAlloc(2 * mesh.triangleCount * sizeof(int32_t));
  physicsBody.faceNormals = MemAlloc(mesh.triangleCount * sizeof(Vector3));
  physicsBody.edges = MemAlloc(2 * 3 * mesh.triangleCount * sizeof(int32_t));
  physicsBody.edgeDirections = MemAlloc(3 * mesh.triangleCount * sizeof(Vector3));
  physicsBody.faceCount = 0;
  physicsBody.faceNormalCount = 0;
  physicsBody.edgeCount = 0;
  physicsBody.edgeDirectionCount = 0;

  float epsilon = 0.0001f;

  for (int32_t i = 0; i < mesh.triangleCount; i++) {
    // Load vertices
    Vector3 vertices[3];
    f_GetTriangleVertices(&mesh, i, vertices);
    Vector3 v1 = vertices[0], v2 = vertices[1], v3 = vertices[2];

    // Process face normal
    Vector3 meshNormal = Vector3Normalize(Vector3CrossProduct(
        Vector3Subtract(v2, v1),
        Vector3Subtract(v3, v1)));

    if (Vector3LengthSqr(meshNormal) < 1e-4f) continue;

    // Find vertex indices
    int32_t vIndices[3] = {-1, -1, -1};
    for (int32_t j = 0; j < physicsBody.vertexCount; j++) {
      if (vIndices[0] < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v1)) < 1e-8f)
        vIndices[0] = j;
      if (vIndices[1] < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v2)) < 1e-8f)
        vIndices[1] = j;
      if (vIndices[2] < 0 && Vector3LengthSqr(Vector3Subtract(physicsBody.vertices[j], v3)) < 1e-8f)
        vIndices[2] = j;
    }

    // Process face
    physicsBody.faces[2 * physicsBody.faceCount] = vIndices[0];
    // Keep the the face normal array duplication free
    // Face normal index references will be offset by one
    // and be negative if the face is facing the opposite direction
    bool duplicatedAxis = false;
    for (int32_t j = 0; j < physicsBody.faceNormalCount; j++) {
      float dot = Vector3DotProduct(meshNormal, physicsBody.faceNormals[j]);
      if (fabs(fabs(dot) - 1.0f) < epsilon) {
        duplicatedAxis = true;
        physicsBody.faces[2 * physicsBody.faceCount + 1] = dot > 0.0f ? j + 1 : -(j + 1);
        break;
      }
    }

    // Process edges
    Vector3 edgeVectors[3] = {
        Vector3Normalize(Vector3Subtract(v2, v1)),
        Vector3Normalize(Vector3Subtract(v3, v2)),
        Vector3Normalize(Vector3Subtract(v1, v3))};

    bool dupEdge[3] = {false}, dupDir[3] = {false};

    // Check for duplicate edges and directions

    for (int32_t j = 0; j < physicsBody.edgeCount; j++) {
      for (int32_t e = 0; e < 3; e++) {
        int32_t idx1 = vIndices[e], idx2 = vIndices[(e + 1) % 3];
        if (!dupEdge[e] && ((idx1 == physicsBody.edges[2 * j] && idx2 == physicsBody.edges[2 * j + 1]) ||
                            (idx2 == physicsBody.edges[2 * j] && idx1 == physicsBody.edges[2 * j + 1]))) {
          dupEdge[e] = true;
        }
      }
    }

    for (int32_t j = 0; j < physicsBody.edgeDirectionCount; j++) {
      for (int e = 0; e < 3; e++) {
        if (!dupDir[e] && (Vector3LengthSqr(Vector3Subtract(physicsBody.edgeDirections[j], edgeVectors[e])) < 1e-4f ||
                           Vector3LengthSqr(Vector3Subtract(Vector3Negate(physicsBody.edgeDirections[j]), edgeVectors[e])) < 1e-4f)) {
          dupDir[e] = true;
        }
      }
    }

    // Add new edges and directions
    for (int e = 0; e < 3; e++) {
      if (!dupEdge[e]) {
        physicsBody.edges[2 * physicsBody.edgeCount] = vIndices[e];
        physicsBody.edges[2 * physicsBody.edgeCount + 1] = vIndices[(e + 1) % 3];
        physicsBody.edgeCount++;
      }
      if (!dupDir[e]) {
        physicsBody.edgeDirections[physicsBody.edgeDirectionCount++] = edgeVectors[e];
      }
    }

    // Update face counts
    if (!duplicatedAxis) {
      physicsBody.faceNormals[physicsBody.faceNormalCount++] = meshNormal;
      physicsBody.faces[2 * physicsBody.faceCount + 1] = physicsBody.faceNormalCount;
    }
    physicsBody.faceCount++;
    triangles[triangleCount++] = (Triangle){v1, v2, v3};
  }

  // Resize arrays
  if (physicsBody.faceNormalCount != mesh.triangleCount)
    physicsBody.faceNormals = MemRealloc(physicsBody.faceNormals, physicsBody.faceNormalCount * sizeof(Vector3));
  if (physicsBody.faceCount != mesh.triangleCount)
    physicsBody.faces = MemRealloc(physicsBody.faces, 2 * physicsBody.faceCount * sizeof(int32_t));
  if (physicsBody.edgeDirectionCount != 3 * mesh.triangleCount)
    physicsBody.edgeDirections = MemRealloc(physicsBody.edgeDirections, physicsBody.edgeDirectionCount * sizeof(Vector3));
  if (physicsBody.edgeCount != 3 * mesh.triangleCount)
    physicsBody.edges = MemRealloc(physicsBody.edges, 2 * physicsBody.edgeCount * sizeof(int32_t));

  // We can clean up the inner edge between two contigious and co-planar triangles like this:
  //     .----.
  //    / \  /
  //   /___\/
  // We can also reduce faces by this method
  // I used some kind of external module here for my own convenience
  for (int32_t i = 0; i < triangleCount; i++) {
    Triangle *coplanarTriangles = MemAlloc(triangleCount * sizeof(Triangle));
    int32_t coplanarTriangleCount = 1;
    coplanarTriangles[0] = triangles[i];

    Vector3 currentRefNormal = GetTriangleNormal(triangles[i]);
    float d = -Vector3DotProduct(triangles[i].p1, currentRefNormal);
    for (int32_t j = 0; j < triangleCount; j++) {
      if (i == j) continue;
      if (fabsf(Vector3DotProduct(triangles[j].p1, currentRefNormal) + d) < 1e-2f && fabsf(Vector3DotProduct(triangles[j].p2, currentRefNormal) + d) < 1e-2f && fabsf(Vector3DotProduct(triangles[j].p3, currentRefNormal) + d) < 1e-2f) {
        coplanarTriangles[coplanarTriangleCount++] = triangles[j];
      }
    }

    Edge *horizon = MemAlloc(3 * triangleCount * sizeof(Edge));
    int32_t horizonCount = 0;
    for (int32_t j = 0; j < coplanarTriangleCount; j++) {
      Edge edges[3] = {
          (Edge){coplanarTriangles[j].p1, coplanarTriangles[j].p2},
          (Edge){coplanarTriangles[j].p2, coplanarTriangles[j].p3},
          (Edge){coplanarTriangles[j].p3, coplanarTriangles[j].p1}};
      for (int32_t l = 0; l < 3; l++) {
        Edge e = edges[l];
        for (int32_t m = 0; m < horizonCount; m++) {
          Edge horizonEdge = horizon[m];
          if (CompareEdges(horizonEdge, e)) {
            horizon[m] = horizon[horizonCount - 1];
            horizonCount--;
            f_RemoveEdge(e, &physicsBody);
          }
        }
        horizon[horizonCount++] = e;
      }
    }

    // Remove coplanar triangles/faces
    for (int32_t j = 1; j < coplanarTriangleCount; j++) {
      int32_t removedIdx = f_RemoveTriangle(&physicsBody, coplanarTriangles[j]);
      triangles[removedIdx] = triangles[triangleCount - 1];
      triangleCount--;
    }

    MemFree(coplanarTriangles);
    MemFree(horizon);
  }
  MemFree(triangles);

  // Must set this, or all 4 component will be zeroes
  physicsBody.rotation = QuaternionIdentity();
  physicsBody.mass = 1.0f;
  physicsBody.inertia = 0.1f;
  physicsBody.colliderType = COLLIDER_TYPE_RIGID;
  physicsBody.colliderShape = COLLIDER_SHAPE_CONVEX;

  return physicsBody;
}

PhysicsBody LoadPhysicsBodySphere(float radius) {
  PhysicsBody physicsBody = {0};

  physicsBody.colliderType = COLLIDER_TYPE_RIGID;
  physicsBody.colliderShape = COLLIDER_SHAPE_SPHERE;

  physicsBody.radius = radius;
  physicsBody.boundingBox = (BoundingBox){
      .min = {-radius, -radius, -radius},
      .max = {radius, radius, radius}};
  // Must set this, or all 4 component will be zeroesc
  physicsBody.rotation = QuaternionIdentity();
  physicsBody.mass = 1.0f;
  physicsBody.inertia = 2.0f / 5.0f * physicsBody.mass * physicsBody.radius * physicsBody.radius;

  return physicsBody;
}

void UnloadPhysicsBody(PhysicsBody physicsBody) {
  MemFree(physicsBody.vertices);
  MemFree(physicsBody.faceNormals);
  MemFree(physicsBody.edgeDirections);
  MemFree(physicsBody.edges);
  MemFree(physicsBody.faces);
}

Manifold CheckCollisionPhysicsBodies(const PhysicsBody *body1, const PhysicsBody *body2) {
  if (body1->colliderType == COLLIDER_TYPE_STATIC && body2->colliderType == COLLIDER_TYPE_STATIC) {
    return (Manifold){0};
  }

  Manifold manifold = {0};
  if (!CheckCollisionBoxes(PhysicsBodyGetWorldBoundingBox(body1), PhysicsBodyGetWorldBoundingBox(body2))) {
    return manifold;
  }

  Matrix body1Matrix = QuaternionToMatrix(body1->rotation);
  Matrix body2Matrix = QuaternionToMatrix(body2->rotation);

  // Collision with spheres
  // If there are two spheres colliding,
  // simply project two extreme points
  if (body1->colliderShape == COLLIDER_SHAPE_SPHERE && body2->colliderShape == COLLIDER_SHAPE_SPHERE) {
    manifold.overlapping = true;
    manifold.depth = f_CheckSphereOverlap(body1, body2);
    manifold.seperationAxis = Vector3Normalize(Vector3Subtract(body2->position, body1->position));
    manifold.collisionType = COLLISION_TYPE_SPHERES;
    return manifold;
  }

  // If a sphere collides with a convex shape
  // we only need to check the axes that come
  // from the convex shape's normals and edges
  if (body1->colliderShape == COLLIDER_SHAPE_SPHERE || body2->colliderShape == COLLIDER_SHAPE_SPHERE) {
    const PhysicsBody *sphere;
    const PhysicsBody *convex;

    if (body1->colliderShape == COLLIDER_SHAPE_SPHERE) {
      sphere = body1;
      convex = body2;
    } else {
      sphere = body2;
      convex = body1;
    }

    float minOverlapDepth = 1e30f;
    Vector3 minOverlapAxis = {0};
    int32_t colType = -1;

    // Axes from convex's normals
    for (int32_t i = 0; i < convex->faceNormalCount; i++) {
      Vector3 axis = Vector3RotateByQuaternion(convex->faceNormals[i], convex->rotation);
      float depth = f_CheckSphereConvexOverlap(sphere, convex, axis);
      if (depth < 0) {
        manifold.seperationAxis = axis;
        return manifold;
      }
      if (depth < minOverlapDepth) {
        minOverlapDepth = depth;
        minOverlapAxis = axis;
        if (sphere == body1) {
          colType = COLLISION_TYPE_SPHERE_TO_FACE;
        } else {
          colType = COLLISION_TYPE_FACE_TO_SPHERE;
        }
      }
    }

    // Axes from convex's edges
    Vector3 nearestEdgeAxis = {0};
    Vector3 overlapPoint = {0};

    f_NearestEdgeAxis(sphere, convex, &nearestEdgeAxis, &overlapPoint);

    manifold.nearestEdgeAxis = nearestEdgeAxis;
    manifold.nearestEdgeOverlap = overlapPoint;

    // The nearest edge axis could be the seperation axis
    float edgeDepth = f_CheckSphereConvexOverlap(sphere, convex, nearestEdgeAxis);
    if (edgeDepth < 0) {
      manifold.seperationAxis = nearestEdgeAxis;
      return manifold;
    }
    if (edgeDepth < minOverlapDepth) {
      minOverlapDepth = edgeDepth;
      minOverlapAxis = nearestEdgeAxis;
      if (sphere == body1) {
        colType = COLLISION_TYPE_SPHERE_TO_EDGE;
      } else {
        colType = COLLISION_TYPE_EDGE_TO_SPHERE;
      }
      // It's easier to fill out the collision data right here
      manifold.contactPointCount = 1;
      manifold.contactNormals[0] = nearestEdgeAxis;
      manifold.contactPoints[0] = overlapPoint;
    }

    manifold.overlapping = true;
    manifold.seperationAxis = minOverlapAxis;
    manifold.depth = minOverlapDepth;
    manifold.collisionType = colType;
    return manifold;
  }

  // Convex vs. Convex

  float minOverlapDepth = 1e30f;
  Vector3 minOverlapAxis = {0};
  int32_t minOverlapCollisionType = COLLISION_TYPE_VERTEX_TO_FACE;

  // Check with body1's normals
  for (int32_t i = 0; i < body1->faceNormalCount; i++) {
    Vector3 axis = Vector3RotateByQuaternion(body1->faceNormals[i], body1->rotation);
    float depth = f_CheckConvexOverlap(body1, body2, axis);
    if (depth < 0) {
      manifold.seperationAxis = axis;
      return manifold;
    }
    if (depth < minOverlapDepth) {
      minOverlapDepth = depth;
      minOverlapAxis = axis;
      minOverlapCollisionType = COLLISION_TYPE_FACE_TO_VERTEX;
    }
  }
  // Check with body2's normals
  for (int32_t i = 0; i < body2->faceNormalCount; i++) {
    Vector3 axis = Vector3RotateByQuaternion(body2->faceNormals[i], body2->rotation);
    float depth = f_CheckConvexOverlap(body1, body2, axis);
    if (depth < 0) {
      manifold.seperationAxis = axis;
      return manifold;
    }
    if (depth < minOverlapDepth) {
      minOverlapDepth = depth;
      minOverlapAxis = axis;
      minOverlapCollisionType = COLLISION_TYPE_VERTEX_TO_FACE;
    }
  }
  // Check with body1 & body2 crossed-product edge
  for (int32_t i = 0; i < body1->edgeDirectionCount; i++) {
    Vector3 body1Edge = Vector3RotateByQuaternion(body1->edgeDirections[i], body1->rotation);
    for (int32_t j = 0; j < body2->edgeDirectionCount; j++) {
      Vector3 body2Edge = Vector3RotateByQuaternion(body2->edgeDirections[j], body2->rotation);

      Vector3 axis = Vector3CrossProduct(body1Edge, body2Edge);
      if (Vector3LengthSqr(axis) <= 1e-4f) {
        continue;
      }
      axis = Vector3Normalize(axis);
      float depth = f_CheckConvexOverlap(body1, body2, axis);
      if (depth < 0) {
        manifold.seperationAxis = axis;
        return manifold;
      }
      if (depth < minOverlapDepth) {
        minOverlapDepth = depth;
        minOverlapAxis = axis;
        minOverlapCollisionType = COLLISION_TYPE_EDGE_TO_EDGE;
      }
    }
  }
  manifold.overlapping = true;
  manifold.seperationAxis = Vector3Normalize(minOverlapAxis);
  manifold.depth = minOverlapDepth;
  manifold.collisionType = minOverlapCollisionType;
  return manifold;
}

void PhysicsBodyApplyGravity(PhysicsBody *physicsBody, float delta) {
  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC){
    return;
  }
  delta *= physicsTimeScale;
  physicsBody->velocity.y -= GRAVITY * delta;
}

void DrawFaceNormals(PhysicsBody physicsBody) {
  for (int32_t i = 0; i < physicsBody.faceNormalCount; i++) {
    Vector3 normal = Vector3RotateByQuaternion(physicsBody.faceNormals[i], physicsBody.rotation);
    Vector3 drawPos = physicsBody.position;
    // Draw axis
    DrawLine3D(Vector3Add(Vector3Scale(Vector3Negate(normal), 1000.0f), drawPos), Vector3Add(Vector3Scale(normal, 1000.0f), drawPos), BLUE);
  }
}

void DrawFaceNormalCoords(PhysicsBody physicsBody, Camera camera) {
  Vector3 camForward = GetCameraForward(&camera);
  for (int i = 0; i < physicsBody.faceNormalCount; i++) {
    Vector3 drawPos = Vector3Add(physicsBody.position, Vector3Scale(physicsBody.faceNormals[i], 2.0f));
    if (Vector3DotProduct(camForward, Vector3Subtract(drawPos, camera.position)) > 0) {
      Vector2 screenPos = GetWorldToScreen(drawPos, camera);
      DrawText(TextFormat("(%.1f, %.1f, %.1f)", physicsBody.faceNormals[i].x, physicsBody.faceNormals[i].y, physicsBody.faceNormals[i].z), (int)screenPos.x + 10, (int)screenPos.y - 4, 4, BLACK);
    }
  }
}

void PhysicsBodyUpdate(PhysicsBody *physicsBody, float delta) {
  delta *= physicsTimeScale;

  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC) {
    physicsBody->angularVelocity = (Vector3){0};
    physicsBody->velocity = (Vector3){0};
    return;
  }

  // Note to self: Don't ever use Vector3Equals() for stopping threshold, e.g Vector3Equals(linearMovement, Vector3Zero())
  // It has a very small epsilon which is more suitable for removing vertex duplicates
  float angularEpsilon = 1e-3f;

  float angularDamping = 0.5f;
  physicsBody->angularVelocity = Vector3Scale(physicsBody->angularVelocity, Clamp(1 - angularDamping * delta, 0, 1));
  
  Vector3 angularMovement = Vector3Scale(physicsBody->angularVelocity, delta);
  if (Vector3LengthSqr(angularMovement) > angularEpsilon * angularEpsilon) {
    physicsBody->rotation = QuaternionMultiply(QuaternionFromEuler(angularMovement.x, angularMovement.y, angularMovement.z), physicsBody->rotation);
  }
  
  float linearHorizontalEpsilon = 1e-3f;

  float linearDamping = 1.0f;
  //float vy = physicsBody->velocity.y;
  physicsBody->velocity = Vector3Scale(physicsBody->velocity, Clamp(1 - linearDamping * delta, 0, 1));
  //physicsBody->velocity.y = vy;

  Vector3 linearMovement = Vector3Scale(physicsBody->velocity, delta);
  Vector3 linearHorizontalMovement = (Vector3){linearMovement.x, 0, linearMovement.z};
  if (Vector3LengthSqr(linearHorizontalMovement) > linearHorizontalEpsilon * linearHorizontalEpsilon) {
    physicsBody->position = Vector3Add(physicsBody->position, linearHorizontalMovement);
  }
  physicsBody->position.y += linearMovement.y;

}

void PhysicsBodyAddImpulse(PhysicsBody *physicsBody, float delta, Vector3 impulse) {
  physicsBody->velocity = Vector3Add(physicsBody->velocity, impulse);
}

void ResolveCollisionPhysicsBodies(PhysicsBody *body1, PhysicsBody *body2, Manifold *manifold, float delta) {
  if (!manifold->overlapping) {
    return;
  }

  // Seperation axis but make it in the direction from body1 -> body2
  Vector3 body1ToBody2SepAxis = Vector3DotProduct(Vector3Subtract(body2->position, body1->position), manifold->seperationAxis) < 0 ? Vector3Negate(manifold->seperationAxis) : manifold->seperationAxis;
  // Seperation axis but make it in the direction from body2 -> body1
  Vector3 body2ToBody1SepAxis = Vector3Negate(body1ToBody2SepAxis);
  float colMargin = 0.0f;  // An extra margin to prevent objects from sticking together again

  // Seperating two objects
  if (body1->colliderType == COLLIDER_TYPE_RIGID && body2->colliderType == COLLIDER_TYPE_RIGID) {
    body1->position = Vector3Add(body1->position, Vector3Scale(body2ToBody1SepAxis, manifold->depth * 0.5f + colMargin));
    body2->position = Vector3Add(body2->position, Vector3Scale(body1ToBody2SepAxis, manifold->depth * 0.5f + colMargin));
  } else if (body1->colliderType == COLLIDER_TYPE_RIGID && body2->colliderType == COLLIDER_TYPE_STATIC) {
    body1->position = Vector3Add(body1->position, Vector3Scale(body2ToBody1SepAxis, manifold->depth + colMargin));
  } else {
    body2->position = Vector3Add(body2->position, Vector3Scale(body1ToBody2SepAxis, manifold->depth + colMargin));
  }
  // For waking up sleeping and floating bodies 
  // body1->hasContact = true;
  // body2->hasContact = true;

  // Find the contact point
  switch (manifold->collisionType) {
    case COLLISION_TYPE_SPHERE_TO_EDGE: {
      // This type of collision got filled out in the collision detection phase
      break;
    }
    case COLLISION_TYPE_EDGE_TO_SPHERE: {
      // This type of collision got filled out in the collision detection phase
      break;
    }
    case COLLISION_TYPE_SPHERE_TO_FACE: {
      Vector3 faceRefPoint = {0};
      Vector3 faceRefNormal = {0};
      // Find the face of body2 that is the most aligned with sep axis
      f_FindMostAlignedFace(body2, body1ToBody2SepAxis, &faceRefPoint, &faceRefNormal);

      manifold->contactPointCount = 1;
      manifold->contactPoints[0] = f_ProjectOntoPlane(body1->position, faceRefPoint, faceRefNormal);
      manifold->contactNormals[0] = faceRefNormal;

      break;
    }
    case COLLISION_TYPE_FACE_TO_SPHERE: {
      Vector3 faceRefPoint = {0};
      Vector3 faceRefNormal = {0};
      // Find the face of body1 that is the most aligned with sep axis
      f_FindMostAlignedFace(body1, body1ToBody2SepAxis, &faceRefPoint, &faceRefNormal);

      manifold->contactPointCount = 1;
      manifold->contactPoints[0] = f_ProjectOntoPlane(body2->position, faceRefPoint, faceRefNormal);
      manifold->contactNormals[0] = faceRefNormal;

      break;
    }
    case COLLISION_TYPE_VERTEX_TO_FACE: 
    case COLLISION_TYPE_FACE_TO_VERTEX: {
      PhysicsBody* bodyVertex;
      PhysicsBody* bodyFace;
      Vector3 bodyFaceToBodyVertexSepAxis;
      if (manifold->collisionType == COLLISION_TYPE_VERTEX_TO_FACE){
        bodyVertex = body1;
        bodyFace = body2;
        bodyFaceToBodyVertexSepAxis = body1ToBody2SepAxis;
      } else {
        bodyVertex = body2;
        bodyFace = body1;
        bodyFaceToBodyVertexSepAxis = body2ToBody1SepAxis;
      }

      Vector3 faceRefPoint = {0};
      Vector3 faceRefNormal = {0};
      // Find the face of bodyFace that is the most aligned with sep axis
      f_FindMostAlignedFace(bodyFace, bodyFaceToBodyVertexSepAxis, &faceRefPoint, &faceRefNormal);

      // Distance from vertcies of bodyVertex to the most aligned face of bodyFace
      float *vertexDistances = MemAlloc(bodyVertex->vertexCount * sizeof(float));

      float minDistance = 1e30f;
      for (int32_t i = 0; i < bodyVertex->vertexCount; i++) {
        Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(bodyVertex->vertices[i], bodyVertex->rotation), bodyVertex->position);
        float distance = fabs(f_DistanceToPlane(vertex, faceRefPoint, faceRefNormal));
        vertexDistances[i] = distance;
        if (distance < minDistance) {
          minDistance = distance;
        }
      }

      manifold->contactPointCount = 0;
      // Add more vertices with approximately the same distance
      for (int32_t i = 0; i < bodyVertex->vertexCount; i++) {
        if (fabs(vertexDistances[i] - minDistance) <= 0.01f) {
          Vector3 vertex = Vector3Add(Vector3RotateByQuaternion(bodyVertex->vertices[i], bodyVertex->rotation), bodyVertex->position);
          manifold->contactPoints[manifold->contactPointCount] = vertex;
          manifold->contactNormals[manifold->contactPointCount] = faceRefNormal;
          manifold->contactPointCount++;
        }
      }
      
      // Filter out vertice that are not overlapping
      if (manifold->contactPointCount > 1){
        for (int32_t i = 0; i < manifold->contactPointCount; i++) {
          bool inside = true;
          Vector3 contactPoint = manifold->contactPoints[i];
          for (int32_t j = 0; j < bodyFace->faceCount; j++) {
            Vector3 normal = bodyFace->faceNormals[abs(bodyFace->faces[2 * j + 1]) - 1];
            if (bodyFace->faces[2 * j + 1] < 0) {
              normal = Vector3Negate(normal);
            }
            normal = Vector3RotateByQuaternion(normal, bodyFace->rotation);
            Vector3 point = Vector3Add(bodyFace->position, Vector3RotateByQuaternion(bodyFace->vertices[bodyFace->faces[2 * j]], bodyFace->rotation));
            float distanceToPlane = f_DistanceToPlane(contactPoint, point, normal);
            if (distanceToPlane > 0.1f) {
              inside = false;
              break;
            }
          }
          if (!inside) {
            manifold->contactPoints[i] = manifold->contactPoints[manifold->contactPointCount - 1];
            manifold->contactNormals[i] = manifold->contactNormals[manifold->contactPointCount - 1];
            manifold->contactPointCount--;
          }
        }
      }

      MemFree(vertexDistances);
      break;
    }
    case COLLISION_TYPE_EDGE_TO_EDGE: {
      // Find the candidates for the colliding edge of body1
      // Choose from the edges that pendicular to the seperation axis
      int32_t body1EdgeCandidatesCount = 0;
      int32_t *body1EdgeCandidates = MemAlloc(2 * body1->edgeCount * sizeof(int32_t));

      for (int32_t i = 0; i < 2 * body1->edgeCount; i += 2) {
        Vector3 v1 = Vector3RotateByQuaternion(body1->vertices[body1->edges[i]], body1->rotation);
        Vector3 v2 = Vector3RotateByQuaternion(body1->vertices[body1->edges[i + 1]], body1->rotation);
        Vector3 edgeDir = Vector3Subtract(v2, v1);
        if (fabs(Vector3DotProduct(edgeDir, manifold->seperationAxis)) < 0.01f) {
          body1EdgeCandidates[2 * body1EdgeCandidatesCount] = body1->edges[i];
          body1EdgeCandidates[2 * body1EdgeCandidatesCount + 1] = body1->edges[i + 1];
          body1EdgeCandidatesCount++;
        }
      }
      // Find the candidates for the colliding edge of body2
      // Choose from the edges that pendicular to the seperation axis
      int32_t body2EdgeCandidatesCount = 0;
      int32_t *body2EdgeCandidates = MemAlloc(2 * body2->edgeCount * sizeof(int32_t));
      for (int32_t i = 0; i < 2 * body2->edgeCount; i += 2) {
        Vector3 v1 = Vector3RotateByQuaternion(body2->vertices[body2->edges[i]], body2->rotation);
        Vector3 v2 = Vector3RotateByQuaternion(body2->vertices[body2->edges[i + 1]], body2->rotation);
        Vector3 edgeDir = Vector3Subtract(v2, v1);
        if (fabs(Vector3DotProduct(edgeDir, manifold->seperationAxis)) < 0.01f) {
          body2EdgeCandidates[2 * body2EdgeCandidatesCount] = body2->edges[i];
          body2EdgeCandidates[2 * body2EdgeCandidatesCount + 1] = body2->edges[i + 1];
          body2EdgeCandidatesCount++;
        }
      }
      // Find the closest edge pair
      float minDistanceSqr = 1e30f;
      Vector3 intersection = {0};
      Vector3 normal = {0};
      for (int32_t i = 0; i < 2 * body1EdgeCandidatesCount; i += 2) {
        Vector3 v1Start = Vector3Add(Vector3RotateByQuaternion(body1->vertices[body1EdgeCandidates[i]], body1->rotation), body1->position);
        Vector3 v1End = Vector3Add(Vector3RotateByQuaternion(body1->vertices[body1EdgeCandidates[i + 1]], body1->rotation), body1->position);
        for (int32_t j = 0; j < 2 * body2EdgeCandidatesCount; j += 2) {
          Vector3 v2Start = Vector3Add(Vector3RotateByQuaternion(body2->vertices[body2EdgeCandidates[j]], body2->rotation), body2->position);
          Vector3 v2End = Vector3Add(Vector3RotateByQuaternion(body2->vertices[body2EdgeCandidates[j + 1]], body2->rotation), body2->position);
          Vector3 intersectionOnV1 = {0};
          float distanceSqr = f_LineToLineDistanceSqr(v1Start, v1End, v2Start, v2End, &intersectionOnV1);
          if (distanceSqr < minDistanceSqr) {
            minDistanceSqr = distanceSqr;
            intersection = intersectionOnV1;
            normal = Vector3CrossProduct(Vector3Subtract(v1End, v1Start), Vector3Subtract(v2End, v2Start));
            if (Vector3LengthSqr(normal) < 1e-4f) {
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
      if (Vector3DotProduct(normal, Vector3Subtract(body2->position, body1->position)) < 0) {
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

  if (physicsTimeScale < 1e-4f) {
    return;
  }

  delta *= physicsTimeScale;
  // Calculate the impulse base on the velocity of center of mass and the angular velocity of each object
  // e : bounciness
  float e = 0.5f;
  Vector3 totalImpulse = {0};
  // Vectors from center of masses to contact point
  for (int32_t i = 0; i < manifold->contactPointCount; i++) {
    Vector3 p = manifold->contactPoints[i];
    Vector3 n = manifold->contactNormals[i];
    Vector3 r1 = Vector3Subtract(p, body1->position);
    Vector3 r2 = Vector3Subtract(p, body2->position);
    // m_eff : Mass effective, approximated for 2 point masses
    float m_eff = InvMass(body1) + InvMass(body2) + Vector3LengthSqr(Vector3CrossProduct(r1, n)) * InvInertia(body1) + Vector3LengthSqr(Vector3CrossProduct(r2, n)) * InvInertia(body2);

    // vR : relative velocity between two point of each of object at the contact point
    Vector3 vR = Vector3Subtract(
        Vector3Add(body1->velocity, Vector3CrossProduct(body1->angularVelocity, r1)),
        Vector3Add(body2->velocity, Vector3CrossProduct(body2->angularVelocity, r2)));
    // Normal impulse
    float jNMag = -(1 + e) * Vector3DotProduct(vR, n) / m_eff;
    Vector3 jN = Vector3Scale(n, jNMag);

    // Tangential impulse
    Vector3 vN = Vector3Scale(n, Vector3DotProduct(vR, n));
    Vector3 vT = Vector3Subtract(vR, vN);

    Vector3 jT = {0};
    
    if (Vector3LengthSqr(vT) >= 1e-8f) {
      Vector3 t = Vector3Normalize(vT);
      // m_eff : Mass effective for tangential
      float m_effT = InvMass(body1) + InvMass(body2) + Vector3LengthSqr(Vector3CrossProduct(r1, t)) * InvInertia(body1) + Vector3LengthSqr(Vector3CrossProduct(r2, t)) * InvInertia(body2);
      float jTMag = -Vector3DotProduct(vR, t) / m_effT;
      // Coulomb limit: |jT| < mu.|jN|
      // where u : friction coeffiecnt, depend on physics materials
      jT = Vector3Scale(t, jTMag);
      float mu = 0.5f; // Friction coefficient
      float jTMax = mu * jNMag * (jTMag >= 0 ? 1 : -1);
      if (fabsf(jTMag) > fabsf(jTMax)) {
        jT = Vector3Scale(t, jTMax);
      }
    }

    // Apply torques
    // On body1
    Vector3 f1 = Vector3Add(jN, jT);
    Vector3 torque1 = Vector3CrossProduct(r1, f1);
    Vector3 angAccelDelta1 = Vector3Scale(torque1, InvInertia(body1) * delta);
    body1->angularVelocity = Vector3Add(body1->angularVelocity, angAccelDelta1);

    // On body2
    Vector3 f2 = Vector3Negate(Vector3Add(jN, jT));
    Vector3 torque2 = Vector3CrossProduct(r2, f2);
    Vector3 angAccelDelta2 = Vector3Scale(torque2, InvInertia(body2) * delta);
    body2->angularVelocity = Vector3Add(body2->angularVelocity, angAccelDelta2);
    
    totalImpulse = Vector3Add(totalImpulse, Vector3Scale(f1, 1.0f / manifold->contactPointCount));
  }

  Vector3 vDelta1 = Vector3Scale(totalImpulse, InvMass(body1));
  Vector3 vDelta2 = Vector3Scale(totalImpulse, -InvMass(body2));
  body1->velocity = Vector3Add(body1->velocity, vDelta1);
  body2->velocity = Vector3Add(body2->velocity, vDelta2);

}

void WakeUp(PhysicsBody *physicsBody) {
  // physicsBody->sleepTimer = 0.0f;
  // physicsBody->isSleeping = false;
}

void ManifoldToString(Manifold *manifold, char *buffer) {
  char colType[100] = "null";
  if (manifold->overlapping) {
    switch (manifold->collisionType) {
      case COLLISION_TYPE_SPHERE_TO_EDGE: {
        TextCopy(colType, "sphere-to-edge");
        break;
      }
      case COLLISION_TYPE_EDGE_TO_SPHERE: {
        TextCopy(colType, "edge-to-sphere");
        break;
      }
      case COLLISION_TYPE_SPHERE_TO_FACE: {
        TextCopy(colType, "sphere-to-face");
        break;
      }
      case COLLISION_TYPE_FACE_TO_SPHERE: {
        TextCopy(colType, "face-to-sphere");
        break;
      }
      case COLLISION_TYPE_EDGE_TO_EDGE: {
        TextCopy(colType, "edge-to-edge");
        break;
      }
      case COLLISION_TYPE_FACE_TO_VERTEX: {
        TextCopy(colType, "face-to-vertex");
        break;
      }
      case COLLISION_TYPE_VERTEX_TO_FACE: {
        TextCopy(colType, "vertex-to-face");
        break;
      }
      default:
        break;
    }
  }

  sprintf(buffer, "Collision: %s\ncontacts: %d", colType, manifold->contactPointCount);
}

BoundingBox PhysicsBodyGetWorldBoundingBox(PhysicsBody *physicsBody) {
  // For spheres, there is no need to rotate the bounding box
  if (physicsBody->colliderShape == COLLIDER_SHAPE_SPHERE) {
    return (BoundingBox){
        .min = Vector3Add(physicsBody->position, physicsBody->boundingBox.min),
        .max = Vector3Add(physicsBody->position, physicsBody->boundingBox.max),
    };
  }

  BoundingBox bb = physicsBody->boundingBox;
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
  for (int32_t i = 0; i < 8; i++) {
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

BoundingBox PhysicsBodyGetWorldBoundingBoxMargin(PhysicsBody *physicsBody, float margin) {
  BoundingBox bb = PhysicsBodyGetWorldBoundingBox(physicsBody);
  bb.min = Vector3Subtract(bb.min, (Vector3){margin, margin, margin});
  bb.max = Vector3Add(bb.max, (Vector3){margin, margin, margin});
  return bb;
}

float InvInertia(PhysicsBody *physicsBody) {
  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC) {
    return 0.0f;
  }
  return 1.0f / physicsBody->inertia;
}

float InvMass(PhysicsBody *physicsBody) {
  if (physicsBody->colliderType == COLLIDER_TYPE_STATIC) {
    return 0.0f;
  }
  return 1.0f / physicsBody->mass;
}

float PhysicsBodyToString(const PhysicsBody *physicsBody, char *buffer) {
  sprintf(buffer, "Body Name:%s\nv:(%.2f, %.2f, %.2f)\nav:(%.2f, %.2f, %.2f)", physicsBody->bodyName,
          physicsBody->velocity.x, physicsBody->velocity.y, physicsBody->velocity.z,
          physicsBody->angularVelocity.x, physicsBody->angularVelocity.y, physicsBody->angularVelocity.z);
}

void DrawPhysicsBodyEdges(PhysicsBody *physicsBody) {
  if (physicsBody->colliderShape != COLLIDER_SHAPE_CONVEX) {
    return;
  }
  for (int32_t i = 0; i < 2 * physicsBody->edgeCount; i += 2) {
    Vector3 v1 = Vector3Add(physicsBody->position, Vector3RotateByQuaternion(physicsBody->vertices[physicsBody->edges[i]], physicsBody->rotation));
    Vector3 v2 = Vector3Add(physicsBody->position, Vector3RotateByQuaternion(physicsBody->vertices[physicsBody->edges[i + 1]], physicsBody->rotation));
    DrawLine3D(v1, v2, PURPLE);
  }
}

void DrawPhysicsBodyVertexIndices(const PhysicsBody *physicsBody, Camera camera) {
  Vector3 camForward = GetCameraForward(&camera);
  for (int i = 0; i < physicsBody->vertexCount; i++) {
    Vector3 drawPos = Vector3Add(physicsBody->position, Vector3RotateByQuaternion(physicsBody->vertices[i], physicsBody->rotation));
    if (Vector3DotProduct(camForward, Vector3Subtract(drawPos, camera.position)) > 0) {
      Vector2 screenPos = GetWorldToScreen(drawPos, camera);
      DrawText(TextFormat("%d", i), (int)screenPos.x + 10, (int)screenPos.y - 4, 8, BLUE);
    }
  }
}

void DrawPhysicsBodyVertices(const PhysicsBody *physicsBody) {
  for (int32_t i = 0; i < physicsBody->vertexCount; i++)
  {
    Vector3 drawPos = Vector3Add(physicsBody->position, Vector3RotateByQuaternion(physicsBody->vertices[i], physicsBody->rotation));
    DrawSphere(drawPos, 0.1f, DARKBLUE);
  }
}
