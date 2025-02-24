#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H
#include "raylib.h"
#include "raymath.h"
#include "doubly_linked_list.h"
#include "geometry.h"
#include "rlgl.h"
#include "rcamera.h"
#include <stdlib.h>
#include <string.h>

#define MAX_INDICES 500
#define MAX_VERTICES 500
#define MAX_TRIANGLES 500
#define GOLDEN_RATIO_PHI 1.61803398875f
#define SQRT_THREE 1.73205080757f

typedef struct MeshTriangle {
  int index1;
  int index2;
  int index3;
} MeshTriangle;


typedef struct MeshEdge {
  int index1;
  int index2;
} MeshEdge;

typedef struct ConvexShape
{
  int vertexCount;
  Vector3* vertices;
  int triangleCount;
  MeshEdge* triangles;
} ConvexShape;

void CreateRandomVertices(Vector3 v[], int n, int seed);
Mesh GenConvexMesh(Vector3 vertices[], int vertexCount);
Mesh GenDodecahedron(float radius);
Mesh GenConvexCube(Vector3 extents);
bool CanSee(Triangle trig, Vector3 p);
void DrawVertices(Vector3 v[], int n);
void DrawVertexCoords(Vector3 v[], int n, Camera camera);
void DrawVertexIndices(Vector3 v[], int n, Camera camera);
void DrawNormals(Mesh mesh);

#endif