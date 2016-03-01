#pragma once

#include <OpenGP/SurfaceMesh/SurfaceMesh.h>

class Remesher
{
public:
    Remesher(OpenGP::SurfaceMesh& m);
    void remesh(OpenGP::Scalar edge_length);

private:
    void split_long_edges();
    void collapse_short_edges();
    void flip_edges();
    void tangential_smoothing();

    bool is_too_long(OpenGP::SurfaceMesh::Vertex v0, OpenGP::SurfaceMesh::Vertex v1) const
    {
        return (points[v0] - points[v1]).norm() > 4.0f / 3.0f * target_edge_length;
    }

    bool is_too_short(OpenGP::SurfaceMesh::Vertex v0, OpenGP::SurfaceMesh::Vertex v1) const
    {
        return (points[v0] - points[v1]).norm() < 4.0f / 3.0f * target_edge_length;
    }

private:
    OpenGP::SurfaceMesh& mesh;
    OpenGP::Scalar target_edge_length;
    OpenGP::SurfaceMesh::Vertex_property<OpenGP::Point> points;
    OpenGP::SurfaceMesh::Vertex_property<OpenGP::Normal> vnormals;
};