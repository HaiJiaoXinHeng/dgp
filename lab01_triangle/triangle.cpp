#include <OpenGP/GL/GlfwWindow.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
using namespace OpenGP;

int main(){
    
    GlfwWindow window("Tetrahedra", 400, 400);
    window.run();
}
