#pragma once
#include <OpenGP/types.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>

//=============================================================================
namespace OpenGP {
//=============================================================================

class NanoflannAdapter;
class SurfaceMeshVerticesKDTree{
    NanoflannAdapter* _adapter = nullptr; ///< internal
    Mat3xN _data;
public:
    SurfaceMeshVerticesKDTree(){}
    ~SurfaceMeshVerticesKDTree();
    void build(SurfaceMesh& mesh);
    SurfaceMesh::Vertex closest_vertex(const Vec3& p);
};

//=============================================================================
} // namespace OpenGP
//=============================================================================

