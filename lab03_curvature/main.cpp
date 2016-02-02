#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include "ArcballWindow.h"
#include "Curvature.h"

using namespace OpenGP;

int main(){
    ArcballWindow window("title", 400, 400);

    SurfaceMesh mesh;
    mesh.read("indorelax.obj");
    CHECK(mesh.is_triangle_mesh());
    mesh.update_face_normals();
    mesh.update_vertex_normals();

    Curvature curvature(mesh);
    curvature.visualize_gauss_curvature();
    curvature.visualize_mean_curvature();
    curvature.visualize_k1_curvature();

    SurfaceMeshRenderShaded mesh_shaded(mesh);
    window.scene.add(mesh_shaded);
    mesh_shaded.colormap_enabled(true);
    mesh_shaded.colormap_set_range(0, 1);

    return window.run();
}
