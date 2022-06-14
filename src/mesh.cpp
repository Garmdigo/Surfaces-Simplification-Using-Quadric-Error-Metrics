#include "mesh.h"
#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace std;

struct Plane {
  double a;
  double b;
  double c;
  double d;
};

Plane makePlaneEquation(const Mesh& mesh, const Triangle& t) {
  const Vector3d& A = mesh.vertices[t.a].point;
  const Vector3d& B = mesh.vertices[t.b].point;
  const Vector3d& C = mesh.vertices[t.c].point;
  Vector3d N = ((B - A).cross(C - A)).normalized();
  double d = -N.dot(A);
  Plane P;
  P.a = N.x();
  P.b = N.y();
  P.c = N.z();
  P.d = d;
  return P;
}

Matrix4d planeMatrix(const Plane& P) {
  Matrix4d Q;
  // Matrix taken from section 5 of the paper
  Q <<
    P.a*P.a, P.a*P.b, P.a*P.c, P.a*P.d,
    P.a*P.b, P.b*P.b, P.b*P.c, P.b*P.d,
    P.a*P.c, P.b*P.c, P.c*P.c, P.c*P.d,
    P.a*P.d, P.b*P.d, P.c*P.d, P.d*P.d;
  return Q;
}

int getVertex(const Mesh& mesh, int i) {
  if (mesh.vertices[i].parent < 0) {
    // Valid vertex
    return i;
  }
  else {
    // Invalid vertex, go to parent
    return getVertex(mesh, mesh.vertices[i].parent);
  }
}

Triangle getTriangle(const Mesh& mesh, int triangle_index) {
  Triangle t = mesh.triangles[triangle_index];
  t.a = getVertex(mesh, t.a);
  t.b = getVertex(mesh, t.b);
  t.c = getVertex(mesh, t.c);
  return t;
}

Vector3d generateNormal(Mesh& mesh, const Triangle& triangle) {
  //ABC triangle
  Vector3d A = mesh.vertices[triangle.a].point;
  Vector3d B = mesh.vertices[triangle.b].point;
  Vector3d C = mesh.vertices[triangle.c].point;
  Vector3d V = B - A;
  Vector3d W = C - A;
  Vector3d normal = V.cross(W);
  if (normal.x() != 0 || normal.y() != 0 || normal.z() != 0) {
    return normal.normalized();
  }
  else {
    return normal;
  }
}

void generateNormals(Mesh& mesh) {
  mesh.normals.resize(mesh.triangles.size());
  for (int i = 0; i < mesh.triangles.size(); ++i) {
    Triangle t = getTriangle(mesh, i);
    mesh.normals[i] = generateNormal(mesh, t);
  }
}

void initializeNeighbors(Mesh& mesh, double radius)
{
  for (int i = 0; i < mesh.vertices.size(); i++)
  {
    for (int j = 0; j < mesh.vertices.size(); j++)
    {
      if (i == j) {
        // skip to avoid comparing a point with itself
        continue;
      }

      assert(mesh.vertices[i].parent == -1);
      assert(mesh.vertices[j].parent == -1);

      Vector3d T1 = mesh.vertices[i].point;
      Vector3d T2 = mesh.vertices[j].point;

      double square_distance = (T1 - T2).squaredNorm();
      if (square_distance < radius * radius)
      {
        mesh.vertices[i].neighbors.push_back(j);
      }
    }
  }
}

// For every vertex find the other vertices that share an edge with the current vertex
void initializeConnectedVertices(Mesh& mesh) {
  for (int t = 0; t < mesh.triangles.size(); ++t) {
    int a = mesh.triangles[t].a;
    int b = mesh.triangles[t].b;
    int c = mesh.triangles[t].c;

    mesh.vertices[a].neighbors.push_back(b);
    mesh.vertices[a].neighbors.push_back(c);

    mesh.vertices[b].neighbors.push_back(a);
    mesh.vertices[b].neighbors.push_back(c);

    mesh.vertices[c].neighbors.push_back(a);
    mesh.vertices[c].neighbors.push_back(b);
  }
}

void removeDuplicates(std::vector<int>& v) {
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
}

void removeDuplicates(Mesh& mesh) {
  for (int i = 0; i < mesh.vertices.size(); i++) {
    removeDuplicates(mesh.vertices[i].neighbors);
  }
}

// For every vertex find its matrix Q
void initializeMatrix(Mesh& mesh) {
  // Initialize matrix Q with 0 for all vertices
  for (int i = 0; i < mesh.vertices.size(); ++i) {
    mesh.vertices[i].Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

  for (int t = 0; t < mesh.triangles.size(); ++t) {
    Plane P = makePlaneEquation(mesh, mesh.triangles[t]);
    Matrix4d Q = planeMatrix(P);
    // Add Q to the 3 vertices of the triangle
    int a = mesh.triangles[t].a;
    int b = mesh.triangles[t].b;
    int c = mesh.triangles[t].c;
    mesh.vertices[a].Q += Q;
    mesh.vertices[b].Q += Q;
    mesh.vertices[c].Q += Q;
  }
}

void recomputeMatrices(Mesh& mesh) {
  // Initialize matrix Q with 0 for all vertices
  for (int i = 0; i < mesh.vertices.size(); ++i) {
    mesh.vertices[i].Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

  for (int t = 0; t < mesh.triangles.size(); ++t) {
    Triangle tri = getTriangle(mesh, t);
    Plane P = makePlaneEquation(mesh, tri);
    Matrix4d Q = planeMatrix(P);
    // Add Q to the 3 vertices of the triangle
    int a = tri.a;
    int b = tri.b;
    int c = tri.c;
    mesh.vertices[a].Q += Q;
    mesh.vertices[b].Q += Q;
    mesh.vertices[c].Q += Q;
  }
}

Mesh loadOff(const char* path, double radius) {
  Mesh mesh;

  std::ifstream input_file(path);
  if (!input_file.is_open()) {
    cout << "ERROR can't open " << path << "\n";
    return mesh;
  }

  string identifier;
  input_file >> identifier;
  if (identifier != "OFF") {
    cout << "ERROR missing OFF\n";
    return mesh;
  }
  int vert_count = 0;
  int face_count = 0;
  int other = 0; // TODO
  input_file >> vert_count >> face_count >> other;

  std::vector<Vector3d> verts;
  Vector3d average = Vector3d(0, 0, 0);
  for (int i = 0; i < vert_count; ++i) {
    Vertex v;
    v.parent = -1;
    input_file >> v.point.x() >> v.point.y() >> v.point.z();
    mesh.vertices.push_back(v);
    average += mesh.vertices.back().point;
  }

  // Center it
  average = average / double(vert_count);
  for (int i = 0; i < vert_count; ++i) {
    mesh.vertices[i].point = mesh.vertices[i].point - average;
  }

  // Scale it to unit cube
  double max_dist = 0;
  for (int i = 0; i < vert_count; ++i) {
    double d = mesh.vertices[i].point.x() * mesh.vertices[i].point.x() + mesh.vertices[i].point.y() * mesh.vertices[i].point.y() + mesh.vertices[i].point.z() * mesh.vertices[i].point.z();
    max_dist = max(d, max_dist);
  }
  if (max_dist > 0) {
    max_dist = sqrt(max_dist);
    for (int i = 0; i < vert_count; ++i) {
      mesh.vertices[i].point = mesh.vertices[i].point / max_dist;
    }
  }

  // Read faces
  for (int i = 0; i < face_count; ++i) {
    int face_vert_count;
    input_file >> face_vert_count;
    if (face_vert_count != 3) {
      cout << "ERROR faces must be triangles\n";
      return mesh;
    }
    Triangle t;
    input_file >> t.a >> t.b >> t.c;
    mesh.triangles.push_back(t);
  }

  generateNormals(mesh);

  // Setup neighbors
  initializeNeighbors(mesh, radius);
  initializeConnectedVertices(mesh);
  removeDuplicates(mesh);
  initializeMatrix(mesh);

  return mesh;
}

Vector3d newPosition(const Mesh& mesh, int p, int q, bool usePaperMethod) {
  if (usePaperMethod) {
    const Matrix4d& Q1 = mesh.vertices[p].Q;
    const Matrix4d& Q2 = mesh.vertices[q].Q;
    Matrix4d Q = Q1 + Q2;
    if (abs(Q.determinant()) < 0.0001) {
      return (mesh.vertices[p].point + mesh.vertices[q].point) * 0.5;
    }
    else {
      Vector4d pos4d = Q.inverse() * Vector4d(0, 0, 0, 1);
      Vector3d pos3d = Vector3d(pos4d.x(), pos4d.y(), pos4d.z());
      if ((pos3d - mesh.vertices[p].point).squaredNorm() > (mesh.vertices[q].point - mesh.vertices[p].point).squaredNorm()) {
        return (mesh.vertices[p].point + mesh.vertices[q].point) * 0.5;
      }
      return pos3d;
    }
  }
  else {
    return (mesh.vertices[p].point + mesh.vertices[q].point) * 0.5;
  }
}

double l2NormError(const Mesh& mesh, int p, int q) {
  const Vector3d& T1 = mesh.vertices[p].point;
  const Vector3d& T2 = mesh.vertices[q].point;
  double square_distance = (T1 - T2).squaredNorm();
  return square_distance;
}

double paperError(const Mesh& mesh, int p, int q) {
  const Matrix4d& Q1 = mesh.vertices[p].Q;
  const Matrix4d& Q2 = mesh.vertices[q].Q;
  Matrix4d Q = Q1 + Q2;
  Vector3d pos3d = newPosition(mesh, p, q, true);
  Vector4d pos4d = Vector4d(pos3d.x(), pos3d.y(), pos3d.z(), 1);
  return pos4d.transpose() * Q * pos4d;
}

// Give a mesh find the 2 nearest vertices and save the index of these 2 vertices in a and b
void findClosest(const Mesh& mesh, int& a, int& b, bool usePaperMethod) {
  double Highest = 999999999;
  for (int i = 0; i < mesh.vertices.size(); i++)
  {
    for (int n = 0; n < mesh.vertices[i].neighbors.size(); n++)
    {
      int j = mesh.vertices[i].neighbors[n];

      int p = getVertex(mesh, i);
      int q = getVertex(mesh, j);

      if (p == q) {
        // skip to avoid comparing a point with itself
        continue;
      }

      double d = 0;
      if (usePaperMethod) {
        d = paperError(mesh, p, q);
      }
      else {
        d = l2NormError(mesh, p, q);
      }
      if (d < Highest)
      {
        Highest = d;
        a = p;
        b = q;
      }
    }
  }
}

void simplify(Mesh& mesh, int step_count, bool usePaperMethod)
{
  std::cout << "Old number of triangles: " << mesh.triangles.size() << "\n";

  for (int i = 0; i < step_count; ++i) {
    int idxA = 0;
    int idxB = 0;
    findClosest(mesh, idxA, idxB, usePaperMethod);
    if (idxA == idxB) {
      break; // Can't simplify anymore
    }
    mesh.vertices[idxA].point = newPosition(mesh, idxA, idxB, usePaperMethod);
    mesh.vertices[idxB].parent = idxA;
    mesh.vertices[idxA].Q = mesh.vertices[idxA].Q + mesh.vertices[idxB].Q;
  }

  TriangleArray triangles;
  for (int i = 0; i < mesh.triangles.size(); ++i) {
    Triangle t = mesh.triangles[i];
    int a = getVertex(mesh, t.a);
    int b = getVertex(mesh, t.b);
    int c = getVertex(mesh, t.c);
    if (a == b || a == c || b == c) {
      continue;
    }
    t.a = a;
    t.b = b;
    t.c = c;
    triangles.push_back(t);
  }
  mesh.triangles = triangles;

  generateNormals(mesh);

  if (usePaperMethod) {
    recomputeMatrices(mesh);
  }

  std::cout << "New number of triangles: " << mesh.triangles.size() << "\n";
}

// Normals are stored per triangle (one normal for every triangle)
// This function generates a normal for each vertex by averaging the normals at each vertex
std::vector<Vector3d> perVertexNormal(const Mesh& mesh) {
  std::vector<Vector3d> normals(mesh.vertices.size());
  // Set the normals to zero
  for (int i = 0; i < normals.size(); ++i) {
    normals[i] = Vector3d(0, 0, 0);
  }
  // For every triangle
  for (int i = 0; i < mesh.triangles.size(); ++i)
  {
    Triangle t = getTriangle(mesh, i);
    // Accumulate the normal of the triangle for the 3 vertices of the triangle
    normals[t.a] += mesh.normals[i];
    normals[t.b] += mesh.normals[i];
    normals[t.c] += mesh.normals[i];
  }
  // Make sure the normals are unit vectors
  for (int i = 0; i < normals.size(); ++i) {
    // Do not normalize a zero vector (Vector3d(0, 0, 0))
    if (normals[i].x() != 0 || normals[i].y() != 0 || normals[i].z() != 0) {
      normals[i].normalize();
    }
  }
  return normals;
}

void Export(const Mesh& mesh, const string& file_name)
{
  ofstream file(file_name);
  if (file.is_open())
  {
    for (int i = 0; i < mesh.vertices.size(); ++i)
    {
      int v = getVertex(mesh, i);
      const Vector3d& p = mesh.vertices[v].point;
      file << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    std::vector<Vector3d> normals = perVertexNormal(mesh);
    for (int i = 0; i < mesh.vertices.size(); ++i)
    {
      const Vector3d& n = normals[i];
      file << "vn " << n.x() << " " << n.y() << " " << n.z() << "\n";
    }
    for (int i = 0; i < mesh.triangles.size(); ++i)
    {
      Triangle t = getTriangle(mesh, i);
      // Make sure to add 1 to the indices because obj files start indexing vertices from 1
      file << "f " << t.a + 1 << " " << t.b + 1 << " " << t.c + 1 << "\n";
    }
  }
  else
    cout << "Unable to open file";
}

int getTriangleCount(const Mesh& mesh) {
  return mesh.triangles.size();
}

Mesh simplifyToTarget(Mesh mesh, int target_triangle_count)
{
  bool usePaperMethod = true;
  int stepCount = 16;
  while (getTriangleCount(mesh) > target_triangle_count) {
    simplify(mesh, stepCount, usePaperMethod);
  }
  return mesh;
}