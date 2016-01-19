#pragma once
#include <OpenGP/types.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>

using namespace OpenGP;

class ImplicitRBF{
private:
    SurfaceMesh& cloud;
    SurfaceMesh::Vertex_property<Vec3> vpoints;
    SurfaceMesh::Vertex_property<Vec3> vnormals;
    Mat3xN  centers_;
    VecN    weights_;
    
public:
    ImplicitRBF(SurfaceMesh& cloud) : cloud(cloud){
        vpoints = cloud.get_vertex_property<Vec3>("v:point");
        vnormals = cloud.get_vertex_property<Vec3>("v:normal");
        optimize_rbf_weights();
    }

    /// Evaluates RBF at position p
    Scalar eval_implicit_at(const Vec3& p) const;

private:
    /// Fit RBF to given constraints
    void optimize_rbf_weights();

    /// Evaluates RBF kernel at point _x, with center at _center
    static Scalar kernel(const Vec3& center, const Vec3& x){
        double r = (x-center).norm();
        return r*r*r;
    }
};
