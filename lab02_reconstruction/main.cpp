#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderCloud.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderVertexNormals.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderShaded.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
#include "ArcballWindow.h"
using namespace OpenGP;

extern void reconstruct(SurfaceMesh& cloud, SurfaceMesh& output, uint resolution);

int main(int argc, char** argv){
    if(argc!=2) mFatal("application requires one parameter! e.g. sphere.obj");
    SurfaceMesh point_cloud;
    bool success = point_cloud.read(argv[1]);
    if(!success) mFatal() << "File not found: " << argv[1];
  
    ArcballWindow window("lab02_reconstruction", 640, 480);

    // HOMEWORK TASKS
    // 1) Compute normals 
    //    - fetch kNN (k~6-10) for each point
    //    - compute the covariance matrix
    //    - extract (unoriented) normal form the eigendecomposition
    //
    // 2) Re-orient normals coherently
    //    - build Reimannian graph and compute weights
    //    - construct minimal spanning tree
    //    - flip the normals by traversing the tree
    
    ///--- Display the input
    SurfaceMeshRenderCloud points_gl(point_cloud);
    SurfaceMeshRenderVertexNormals normals_gl(point_cloud);
    window.scene.add(points_gl);
    window.scene.add(normals_gl);
    
    ///--- Compute reconstruction
    SurfaceMesh output;
    reconstruct(point_cloud, output, 30 /*grid resolution*/);
    output.update_face_normals();
    output.update_vertex_normals();

    ///--- Renders reconstruction 
    SurfaceMeshRenderFlat mesh_gl(output);
    // SurfaceMeshRenderShaded mesh_gl(output);
    window.scene.add(mesh_gl); 
    
    return window.run();
}
