#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderCloud.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>

#include "ArcballWindow.h"
#include "SurfaceReconstruction.h"

using namespace OpenGP;
int main(int argc, char** argv){
    if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
    SurfaceMesh point_cloud;
    bool success = point_cloud.read(argv[1]);
    if(!success) mFatal() << "File not found: " << argv[1];

    ArcballWindow window("lab02_reconstruction", 640, 480);
    
    ///--- Display the input
    SurfaceMeshRenderCloud points_gl(point_cloud);
    SurfaceMeshRenderVertexNormals normals_gl(point_cloud);
    window.scene.add(points_gl);
    window.scene.add(normals_gl);
    
    ///--- Compute reconstruction
    SurfaceMesh mesh;
    SurfaceReconstruction recon(point_cloud);
    recon.reconstruct(mesh, 30 /*grid resolution*/);
    mesh.update_face_normals();
    mesh.update_vertex_normals();

    return window.run();
}
