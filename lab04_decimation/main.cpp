#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
#include "Decimator.h"
using namespace OpenGP;

struct MainWindow : public ArcballWindow{
    SurfaceMesh mesh;
    Decimator decimator = Decimator(mesh);
    SurfaceMeshRenderFlat renderer = SurfaceMeshRenderFlat(mesh);
 
    MainWindow(int argc, char** argv) : ArcballWindow(__FILE__,400,400){
        if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
        bool success = mesh.read(argv[1]);
        if(!success) mFatal() << "File not found: " << argv[1];
        mesh.update_face_normals(); ///< shading
        this->scene.add(renderer);
        decimator.init();
    }
    
    void key_callback(int key, int scancode, int action, int mods) override{
        ArcballWindow::key_callback(key, scancode, action, mods);
        if(key==GLFW_KEY_SPACE && action==GLFW_RELEASE){
            decimator.exec( .9*mesh.n_vertices() );
            mesh.update_face_normals();
            renderer.init_data();
        }
    }
};

int main(int argc, char** argv){
    MainWindow window(argc, argv);
    return window.run();
}
