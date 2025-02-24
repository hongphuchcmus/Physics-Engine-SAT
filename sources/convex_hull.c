#include "convex_hull.h"

static MeshTriangle *fNewMeshTriangle(int a, int b, int c)
{
  MeshTriangle *triangle = (MeshTriangle *)MemAlloc(sizeof(MeshTriangle));
  *triangle = (MeshTriangle){a, b, c};
  return triangle;
}

static MeshEdge *fNewMeshEdge(int a, int b)
{
  MeshEdge *edge = (MeshEdge *)MemAlloc(sizeof(MeshEdge));
  *edge = (MeshEdge){a, b};
  return edge;
}

static Triangle fMeshTriangleToTriangle(MeshTriangle triangle, Vector3 vertices[])
{
  return (Triangle){
      vertices[triangle.index1],
      vertices[triangle.index2],
      vertices[triangle.index3]};
}

static Edge fConvexShapeEdgeToEdge(MeshEdge *edge, Vector3 vertices[])
{
  return (Edge){
      vertices[edge->index1],
      vertices[edge->index2]};
}

static void fTryAddHorizonEdgeIfUnique(Vector3 vertices[], int vertexCount, DoublyLinkedList *horizon, DoublyLinkedList *triangles, MeshEdge *edgeToAdd)
{
  // Check if an edge is shared by two triangles
  DNode *current = horizon->head;
  while (current)
  {
    MeshEdge *currentEdge = (MeshEdge *)current->data;
    Edge edge1 = fConvexShapeEdgeToEdge(currentEdge, vertices);
    Edge edge2 = fConvexShapeEdgeToEdge(edgeToAdd, vertices);
    if (CompareEdges(edge1, edge2))
    {
      DListRemoveNode(horizon, current); // Edge is shared, remove from horizon
      return;
    }
    current = current->next;
  }

  // Otherwise, add it to the horizon
  DListPushBack(horizon, (void *)edgeToAdd); // edgeToAdd is already allocated
}

static void fIncrementalConvexHull(Vector3 vertices[], int verticeCount, DoublyLinkedList *triangles, int newVertexIndex)
{
  // The horizon stores the edges surrounding the visible triangles
  DoublyLinkedList *horizon = DListNew();
  // Loop through all triangles and check if the new vertex can "see" them
  // If it can, the triangle should be removed
  DNode *current = triangles->head;
  while (current != NULL)
  {
    MeshTriangle *indexedTrig = (MeshTriangle *)current->data;
    Triangle trig = fMeshTriangleToTriangle(*indexedTrig, vertices);
    if (CanSee(trig, vertices[newVertexIndex]))
    {
      // If the edge is not share, it is part of the horizon
      fTryAddHorizonEdgeIfUnique(vertices, verticeCount, horizon, triangles, fNewMeshEdge(indexedTrig->index1, indexedTrig->index2));
      fTryAddHorizonEdgeIfUnique(vertices, verticeCount, horizon, triangles, fNewMeshEdge(indexedTrig->index2, indexedTrig->index3));
      fTryAddHorizonEdgeIfUnique(vertices, verticeCount, horizon, triangles, fNewMeshEdge(indexedTrig->index3, indexedTrig->index1));

      DNode *temp = current;
      current = current->next;
      DListRemoveNode(triangles, temp);
    }
    else
    {
      current = current->next;
    }
  }

  // Form new triangles with the horizon edges
  // The correct order is guaranteed by adding the new vertex as the last vertex
  current = horizon->head;
  while (current)
  {
    MeshEdge *indexedEdge = (MeshEdge *)current->data;
    MeshTriangle *newTrig = fNewMeshTriangle(indexedEdge->index1, indexedEdge->index2, newVertexIndex);
    DListPushBack(triangles, newTrig);
    current = current->next;
  }

  // Free the horizon
  DListClear(horizon);
  MemFree(horizon);
}

void CreateRandomVertices(Vector3 v[], int n, int seed)
{
  int maxExtent = 5;

  SetRandomSeed(seed);
  for (int i = 0; i < n; i++)
  {
    v[i] = (Vector3){
        GetRandomValue(-100, 100) / 100.0f * maxExtent,
        GetRandomValue(-100, 100) / 100.0f * maxExtent,
        GetRandomValue(-100, 100) / 100.0f * maxExtent};
  }
}

Mesh GenConvexMesh(Vector3 vertices[], int vertexCount)
{
  // For debugging purposes
  int buildStep = -1;

  Mesh mesh = {0};

  if (vertexCount < 4)
  {
    return mesh;
  }

  // Object ownership, since Mesh also maintains an array of vertices

  DoublyLinkedList *triangles = DListNew();

  // Form the initial tetrahedron
  Vector3 a = vertices[0];
  Vector3 b = vertices[1];
  Vector3 c = vertices[2];

  Vector3 ab = Vector3Subtract(b, a);
  Vector3 ac = Vector3Subtract(c, a);

  Vector3 crossProduct = Vector3CrossProduct(ab, ac);

  int dIndex = -1;
  for (int i = 3; i < vertexCount; i++)
  {
    Vector3 candidate = vertices[i];
    float dot = Vector3DotProduct(Vector3Subtract(candidate, a), crossProduct);
    if (dot < -EPSILON)
    {
      // When D is behind the ABC plane
      dIndex = i;
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, 1, 2)); // ABC
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, 2, i)); // ACD
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, i, 1)); // ADB
      DListPushBack(triangles, (void *)fNewMeshTriangle(1, i, 2)); // BDC
      break;
    }
    else if (dot > EPSILON)
    {
      // When D is in front of the ABC plane
      dIndex = i;
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, 2, 1)); // ACB
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, i, 2)); // ADC
      DListPushBack(triangles, (void *)fNewMeshTriangle(0, 1, i)); // ABD
      DListPushBack(triangles, (void *)fNewMeshTriangle(1, 2, i)); // BCD
      break;
    }
  }

  if (dIndex < 0)
  {
    return mesh;
  }

  int step = 1;
  // Add new vertices and form new convex hull everytime
  for (int i = 3; i < vertexCount; i++)
  {
    if (i == dIndex)
    {
      continue;
    }

    if (step == buildStep){
      break;
    }
    fIncrementalConvexHull(vertices, vertexCount, triangles, i);
    step++;
  }
  int triangleCount = 0;
  MeshTriangle *triangleArray = DListToArray(triangles, sizeof(MeshTriangle), &triangleCount);
  mesh.vertices = (float *)MemAlloc(triangleCount * 3 * 3 * sizeof(float));
  mesh.normals = (float *)MemAlloc(triangleCount * 3 * 3 * sizeof(float));
  mesh.vertexCount = triangleCount * 3;
  mesh.triangleCount = triangleCount;

  for (int k = 0; k < mesh.triangleCount; k++)
  {
    Triangle triangle = fMeshTriangleToTriangle(triangleArray[k], vertices);

    mesh.vertices[k * 9] = triangle.p1.x;
    mesh.vertices[k * 9 + 1] = triangle.p1.y;
    mesh.vertices[k * 9 + 2] = triangle.p1.z;

    mesh.vertices[k * 9 + 3] = triangle.p2.x;
    mesh.vertices[k * 9 + 4] = triangle.p2.y;
    mesh.vertices[k * 9 + 5] = triangle.p2.z;

    mesh.vertices[k * 9 + 6] = triangle.p3.x;
    mesh.vertices[k * 9 + 7] = triangle.p3.y;
    mesh.vertices[k * 9 + 8] = triangle.p3.z;

    Vector3 trigNormal = GetTriangleNormal(triangle);
    mesh.normals[k * 9] = trigNormal.x;
    mesh.normals[k * 9 + 1] = trigNormal.y;
    mesh.normals[k * 9 + 2] = trigNormal.z;

    mesh.normals[k * 9 + 3] = trigNormal.x;
    mesh.normals[k * 9 + 4] = trigNormal.y;
    mesh.normals[k * 9 + 5] = trigNormal.z;

    mesh.normals[k * 9 + 6] = trigNormal.x;
    mesh.normals[k * 9 + 7] = trigNormal.y;
    mesh.normals[k * 9 + 8] = trigNormal.z;
  }
  UploadMesh(&mesh, false);

  // Free memory
  DListClear(triangles);
  MemFree(triangles);

  return mesh;
}

Mesh GenDodecahedron(float radius)
{
  // 20 vertices of a dodecahedron
  // (±1, ±1, ±1) 8 combinations
  // (0, ±1/φ, ±φ) 4 combinations
  // (±1/φ, ±φ, 0) 4 combinations
  // (±φ, 0, ±1/φ) 4 combinations
  // Unit radius of the containing sphere is √3
  Vector3 vertices[20] = {
      (Vector3){-1, -1, -1},
      (Vector3){-1, -1, 1},
      (Vector3){-1, 1, -1},
      (Vector3){-1, 1, 1},
      (Vector3){1, -1, -1},
      (Vector3){1, -1, 1},
      (Vector3){1, 1, -1},
      (Vector3){1, 1, 1},

      (Vector3){0, -1 / GOLDEN_RATIO_PHI, -GOLDEN_RATIO_PHI},
      (Vector3){0, -1 / GOLDEN_RATIO_PHI, GOLDEN_RATIO_PHI},
      (Vector3){0, 1 / GOLDEN_RATIO_PHI, -GOLDEN_RATIO_PHI},
      (Vector3){0, 1 / GOLDEN_RATIO_PHI, GOLDEN_RATIO_PHI},

      (Vector3){-1 / GOLDEN_RATIO_PHI, -GOLDEN_RATIO_PHI, 0},
      (Vector3){-1 / GOLDEN_RATIO_PHI, GOLDEN_RATIO_PHI, 0},
      (Vector3){1 / GOLDEN_RATIO_PHI, -GOLDEN_RATIO_PHI, 0},
      (Vector3){1 / GOLDEN_RATIO_PHI, GOLDEN_RATIO_PHI, 0},

      (Vector3){-GOLDEN_RATIO_PHI, 0, -1 / GOLDEN_RATIO_PHI},
      (Vector3){-GOLDEN_RATIO_PHI, 0, 1 / GOLDEN_RATIO_PHI},
      (Vector3){GOLDEN_RATIO_PHI, 0, -1 / GOLDEN_RATIO_PHI},
      (Vector3){GOLDEN_RATIO_PHI, 0, 1 / GOLDEN_RATIO_PHI}};
  for (int32_t i = 0; i < 20; i++)
  {
    vertices[i] = Vector3Scale(vertices[i], 1.0f / SQRT_THREE * radius); 
  }
  
  return GenConvexMesh(vertices, 20);
}

Mesh GenConvexCube(Vector3 extents)
{
  Vector3 vertices[8] = {
    (Vector3){-extents.x, -extents.y, -extents.z}, // 0: Left-Bottom-Back
    (Vector3){ extents.x, -extents.y, -extents.z}, // 1: Right-Bottom-Back
    (Vector3){ extents.x,  extents.y, -extents.z}, // 2: Right-Top-Back
    (Vector3){-extents.x,  extents.y, -extents.z}, // 3: Left-Top-Back
    (Vector3){-extents.x, -extents.y,  extents.z}, // 4: Left-Bottom-Front
    (Vector3){ extents.x, -extents.y,  extents.z}, // 5: Right-Bottom-Front
    (Vector3){ extents.x,  extents.y,  extents.z}, // 6: Right-Top-Front
    (Vector3){-extents.x,  extents.y,  extents.z}  // 7: Left-Top-Front
  };

  return GenConvexMesh(vertices, 8);
}

bool CanSee(Triangle trig, Vector3 p)
{
  Vector3 a = trig.p1;
  Vector3 b = trig.p2;
  Vector3 c = trig.p3;
  float epsilon = 0.001f;

  // Not normalized face-normal
  Vector3 normal = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
  Vector3 ap = Vector3Subtract(p, a);

  return Vector3DotProduct(normal, ap) > epsilon;
}

void DrawVertices(Vector3 v[], int n)
{
  for (int i = 0; i < n; i++)
  {
    DrawSphere(v[i], 0.05f, BLACK);
  }
}

void DrawVertexCoords(Vector3 v[], int n, Camera camera)
{
  Vector3 camForward = GetCameraForward(&camera);
  for (int i = 0; i < n; i++)
  {
    if (Vector3DotProduct(camForward, Vector3Subtract(v[i], camera.position)) > 0)
    {
      Vector2 screenPos = GetWorldToScreen(v[i], camera);
      DrawText(TextFormat("%.2f, %.2f, %.2f", v[i].x, v[i].y, v[i].z), (int)screenPos.x + 10, (int)screenPos.y - 4, 8, BLACK);
    }
  }
}

void DrawVertexIndices(Vector3 v[], int n, Camera camera)
{
  Vector3 camForward = GetCameraForward(&camera);
  for (int i = 0; i < n; i++)
  {
    if (Vector3DotProduct(camForward, Vector3Subtract(v[i], camera.position)) > 0)
    {
      Vector2 screenPos = GetWorldToScreen(v[i], camera);
      DrawText(TextFormat("%d", i), (int)screenPos.x + 10, (int)screenPos.y - 4, 8, BLUE);
    }
  }
}

void DrawNormals(Mesh mesh)
{
  for (int i = 0; i < mesh.triangleCount; i++)
  {
    Vector3 cenPos = Vector3Add(Vector3Add((Vector3){mesh.vertices[i * 9], mesh.vertices[i * 9 + 1], mesh.vertices[i * 9 + 2]},
                                           (Vector3){mesh.vertices[i * 9 + 3], mesh.vertices[i * 9 + 4], mesh.vertices[i * 9 + 5]}),
                                (Vector3){mesh.vertices[i * 9 + 6], mesh.vertices[i * 9 + 7], mesh.vertices[i * 9 + 8]});
    cenPos = Vector3Scale(cenPos, 1.0f / 3.0f);
    Vector3 normal = (Vector3){mesh.normals[i * 9], mesh.normals[i * 9 + 1], mesh.normals[i * 9 + 2]};
    DrawSphere(cenPos, 0.02f, GREEN);
    DrawLine3D(cenPos, Vector3Add(cenPos, Vector3Scale(normal, 0.2f)), GREEN);
  }
}
