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

void DrawCubeBoundingBox(BoundingBox box, Color color, Color outlineColor){
  Vector3 position = Vector3Scale(Vector3Add(box.min, box.max), 0.5f);
  Vector3 extents = (Vector3) {box.max.x - position.x, box.max.y - position.y, box.max.z - position.z };
  DrawCube(position, extents.x * 2, extents.y * 2, extents.z * 2, color);
  DrawCubeWires(position, extents.x * 2, extents.y * 2, extents.z * 2, outlineColor);
}

void DrawCubeBoundingBoxPadding(BoundingBox box, Color color, Color outlineColor, float padding){
  Vector3 position = Vector3Scale(Vector3Add(box.min, box.max), 0.5f);
  Vector3 extents = (Vector3) {box.max.x - padding - position.x, box.max.y - padding - position.y , box.max.z - padding - position.z  };
  DrawCube(position, extents.x * 2, extents.y * 2, extents.z * 2, color);
  DrawCubeWires(position, extents.x * 2, extents.y * 2, extents.z * 2, outlineColor);
}

BoundingBox GetBoundingBoxMargin(BoundingBox box, float margin) {
  return (BoundingBox){
    .min = Vector3SubtractValue(box.min, margin),
    .max = Vector3AddValue(box.max, margin)
  };
}
