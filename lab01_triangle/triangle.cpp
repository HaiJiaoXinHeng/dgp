#include <OpenGP/GL/GlfwWindow.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include <OpenGP/SurfaceMesh/GL/SurfaceMeshRenderFlat.h>
using namespace OpenGP;

int main(){
    SurfaceMesh mesh;
    
    SurfaceMesh::Vertex v1 = mesh.add_vertex(Vec3(0, 0, 0));
    SurfaceMesh::Vertex v2 = mesh.add_vertex(Vec3(1, 0, 0));
    SurfaceMesh::Vertex v3 = mesh.add_vertex(Vec3(0, 1, 0));
    mesh.add_triangle(v1, v2, v3);

    mesh.update_face_normals();

    // Now let's scale a single vertex.
    Point pos = mesh.position(v2);
    mesh.position(v2) = pos * 2;

    mesh.position(v2) = pos / 2;

    // Now let's scale the whole mesh.
    SurfaceMesh::Vertex_iterator vIt, vBegin, vEnd;
    vBegin = mesh.vertices_begin();
    vEnd = mesh.vertices_end();
    for (vIt = vBegin; vIt != vEnd; ++vIt){
        SurfaceMesh::Vertex v = *vIt;
        // And now we can scale it as before.
        Point pos = mesh.position(v);
        mesh.position(v) = pos * 2;
    }

    // A better way.
    for(const auto& vertex : mesh.vertices()){
        Point ps = mesh.position(vertex);
        mesh.position(vertex) = ps / 2;
    }

    GlfwWindow window("Tetrahedra", 400, 400);
    SurfaceMeshRenderFlat render(mesh);
    window.scene.add(render);
    window.run();
}
