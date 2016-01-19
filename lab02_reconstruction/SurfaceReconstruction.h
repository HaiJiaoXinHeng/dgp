#pragma once
#include <OpenGP/types.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include "SurfaceMeshVerticesKDTree.h"

using namespace OpenGP;
class SurfaceReconstruction{
private:
    SurfaceMesh& mesh; 
    SurfaceMesh::Vertex_property<Vec3> vpoints;
    SurfaceMesh::Vertex_property<Vec3> vnormals;
    SurfaceMeshVerticesKDTree accelerator;
public:
    SurfaceReconstruction(SurfaceMesh& mesh) : mesh(mesh){
        vpoints = mesh.get_vertex_property<Vec3>("v:point");
        vnormals = mesh.get_vertex_property<Vec3>("v:normal");
    }

public:
    void reconstruct(SurfaceMesh& output, uint resolution);
    Scalar eval_implicit_at(const Vec3& p);
};
