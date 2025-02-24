#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include "raylib.h"

typedef struct Triangle {
  Vector3 p1;
  Vector3 p2;
  Vector3 p3;
} Triangle;

typedef struct Edge {
  Vector3 p1;
  Vector3 p2;
} Edge;

int CompareEdges(Edge a, Edge b);
Vector3 GetTriangleNormal(Triangle triangle);

#endif