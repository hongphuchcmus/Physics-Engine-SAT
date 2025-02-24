#include "geometry.h"
#include "raymath.h"

int CompareEdges(Edge a, Edge b)
{
  if ((Vector3Equals(a.p1, b.p1) && Vector3Equals(a.p2, b.p2))
  || (Vector3Equals(a.p1, b.p2) && Vector3Equals(a.p2, b.p1))){
    return 1;
  }
  return 0;
}

Vector3 GetTriangleNormal(Triangle triangle)
{
  Vector3 v1 = Vector3Subtract(triangle.p2, triangle.p1);
  Vector3 v2 = Vector3Subtract(triangle.p3, triangle.p1);
  Vector3 normal = Vector3CrossProduct(v1, v2);
  // Normalize
  normal = Vector3Normalize(normal);
  return normal;
}
