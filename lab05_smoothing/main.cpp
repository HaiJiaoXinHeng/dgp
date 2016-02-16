#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "Smoother.h"

using namespace OpenGP;

struct MainWindow : public ArcballWindow
{
    SurfaceMesh mesh;
    Smoother smoother = Smoother(mesh);
    SurfaceMeshRenderFlat renderer = SurfaceMeshRenderFlat(mesh);

    MainWindow(int argc, char** argv) : ArcballWindow(__FILE__, 400, 400)
    {
        mesh.read("indorelax.obj");
        CHECK(mesh.is_triangle_mesh());
        mesh.update_face_normals();
        mesh.update_vertex_normals();

        smoother.init();

        this->scene.add(renderer);
        smoother.use_graph_laplacian();
        //smoother.use_cotan_laplacian();
    }

    void key_callback(int key, int scancode, int action, int mods) override
    {
        ArcballWindow::key_callback(key, scancode, action, mods);
        if (key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
        {
            smoother.smooth_explicit(0.01f);
            //smoother.smooth_implicit(0.01f);
            mesh.update_face_normals();
            renderer.init_data();
        }
    }
};

int main(int argc, char** argv){
    MainWindow window(argc, argv);
    return window.run();
}
