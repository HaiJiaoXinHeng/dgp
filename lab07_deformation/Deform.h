#pragma once

#include <OpenGP/SurfaceMesh/SurfaceMesh.h>
#include <OpenGP/Gl/SceneGraph.h>
#include <OpenGP/GL/PointsRenderer.h>
#include <Eigen/Sparse>

#define QUAD_MESH 0
#define WOODY !QUAD_MESH

class Deform
{
public:
    Deform(OpenGP::SurfaceMesh& mesh, OpenGP::SceneGraph& scene);
    ~Deform();

    void construct_handles();
    void intialize_laplacian();
    void select_handle(int index);
    void mouse_down(OpenGP::Point const& pos);
    void mouse_up();

private:
    Eigen::SparseMatrix<OpenGP::Scalar> construct_squared_laplacian();
    void construct_permutation_matrix();

    void factor_matrices();
    void smooth_mesh();
    void compute_displacements();

    void update_mesh();
    void update_mesh_with_displacements();

    OpenGP::SurfaceMesh& mesh;
    OpenGP::SceneGraph& scene;
    OpenGP::PointsRenderer handleRenderer;
    OpenGP::SurfaceMesh::Vertex_property<OpenGP::Point> vpoint;
    OpenGP::SurfaceMesh::Vertex_property<int> vhandle;

    int selected_handle;
    int u, k;

    std::vector<OpenGP::Vec3> displacements;

    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permute;
    Eigen::MatrixXf v_u;
    Eigen::MatrixXf v_k;
    Eigen::SparseMatrix<OpenGP::Scalar> L_uu;
    Eigen::SparseMatrix<OpenGP::Scalar> L_uk;
};