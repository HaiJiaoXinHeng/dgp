#pragma once
#include <OpenGP/types.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include "SurfaceMeshVerticesKDTree.h"

using namespace OpenGP;

class ImplicitHoppe{
    SurfaceMeshVerticesKDTree accelerator;
    SurfaceMesh::Vertex_property<Vec3> vpoints;
    SurfaceMesh::Vertex_property<Vec3> vnormals;
    
public:
    ImplicitHoppe(SurfaceMesh& cloud) : accelerator(cloud){
        vpoints = cloud.get_vertex_property<Vec3>("v:point");
        vnormals = cloud.get_vertex_property<Vec3>("v:normal");        
    }

    /// Evaluates implicit function at position p
    Scalar eval_implicit_at(const Vec3& p) const{
        // COmpute closest point O(log n)
        SurfaceMesh::Vertex closest = accelerator.closest_vertex(p);

        // Compute distance field w.r.t plane at the grid point
        Vec3 normal = vnormals[closest];
        Vec3 point = vpoints[closest];
        return (p - point).dot(normal);
    }
};
