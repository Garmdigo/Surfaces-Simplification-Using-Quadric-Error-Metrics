#pragma once

#include <vector>
#include <string>

// Linear Algebra Library
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

struct Vertex {
	Vector3d point;
	int parent = -1; // when negative point is valid and this parent index isn't used. Otherwise don't use point and got to parent
	std::vector<int> neighbors; // List of indices of vertices to compare against in the function findClosest
	Matrix4d Q; // Q matrix as defined in the paper
};

// Array of vertices
using VertexArray = std::vector<Vertex>;

struct Triangle {
	int a;
	int b;
	int c;
};

using TriangleArray = std::vector<Triangle>;

using NormalArray = std::vector<Vector3d>;

struct Mesh {
	VertexArray vertices;
	TriangleArray triangles;
	NormalArray normals; // One per triangle
};

Mesh loadOff(const char* path, double radius);

void generateNormals(Mesh& mesh);

Triangle getTriangle(const Mesh& mesh, int triangle_index);

int getTriangleCount(const Mesh& mesh);

void simplify(Mesh& mesh, int step_count, bool usePaperMethod);

void Export(const Mesh& mesh, const std::string& file_name);

Mesh simplifyToTarget(Mesh mesh, int target_triangle_count);