#pragma once
#include <vector>
#include <OpenGP/GL/SceneObject.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>

//=============================================================================
namespace OpenGP {
//=============================================================================

class SurfaceMeshRenderShaded : public SceneObject{
private:
    SurfaceMesh& mesh;
    VertexArrayObject vao;    
    ArrayBuffer<Vec3> v_buffer;
    ArrayBuffer<Vec3> n_buffer;
    ArrayBuffer<float> q_buffer;
    ElementArrayBuffer<uint> i_buffer;
    GLuint _tex; ///< Colormap Texture ID
    
public:
    SurfaceMeshRenderShaded(SurfaceMesh& mesh) : mesh(mesh){}
    HEADERONLY_INLINE void init();
    HEADERONLY_INLINE void display();
    
/// @{ color quality mapping
public:
    HEADERONLY_INLINE void colormap_enabled(bool);
    HEADERONLY_INLINE void colormap_set_range(Scalar _min, Scalar _max);
/// @}
};
    
//=============================================================================
} // namespace OpenGP
//=============================================================================

#ifdef HEADERONLY
    #include "SurfaceMeshRenderShaded.cpp"
#endif
