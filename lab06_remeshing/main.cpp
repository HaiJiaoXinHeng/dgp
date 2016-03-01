#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "Remesher.h"

using namespace OpenGP;

struct MainWindow : public ArcballWindow
{
    SurfaceMesh mesh;
    Remesher remesher = Remesher(mesh);
    SurfaceMeshRenderFlat renderer = SurfaceMeshRenderFlat(mesh);

    MainWindow(int argc, char** argv) : ArcballWindow(__FILE__, 400, 400)
    {
        mesh.read("indorelax.obj");
        CHECK(mesh.is_triangle_mesh());
        mesh.update_face_normals();
        mesh.update_vertex_normals();

        this->scene.add(renderer);
    }

    void key_callback(int key, int scancode, int action, int mods) override
    {
        ArcballWindow::key_callback(key, scancode, action, mods);
        if (key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
        {
            remesher.remesh(0.01f);
            mesh.update_face_normals();
            renderer.init_data();
        }
    }
};

int main(int argc, char** argv){
    MainWindow window(argc, argv);
    return window.run();
}
