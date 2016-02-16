#include "Smoother.h"

using namespace Eigen;
using namespace OpenGP;

Smoother::Smoother(OpenGP::SurfaceMesh& mesh) :
    mesh(mesh)
{ }

Smoother::~Smoother()
{ }

void Smoother::init()
{
    int n = mesh.n_vertices();
    L = SparseMatrix<Scalar>(n, n);
}

void Smoother::use_cotan_laplacian()
{
    unsigned int n = mesh.n_vertices();
    typedef Eigen::Triplet<Scalar> Triplet;

    std::vector<Triplet> tripletList;
    tripletList.reserve(n * n);

    // TODO: Fill in the code to compute the cotan laplacian matrix.
    // \li First, compute the cotans of the corresponding angles and the area.
    // \li Second, store them in the appropriate entries of the matrices D and M.
    // \li Last, multiply D * M to get the laplacian.

    for (auto const& v_i : mesh.vertices())
    {
        for (auto const& edge : mesh.halfedges(v_i))
        {
            auto v_j = mesh.to_vertex(edge);
            tripletList.push_back(Triplet(v_i.idx(), v_j.idx(), 0));
        }

        tripletList.push_back(Triplet(v_i.idx(), v_i.idx(), 1));
    }

    L.setFromTriplets(tripletList.begin(), tripletList.end());
}

void Smoother::use_graph_laplacian()
{
    // Grab the number of vertices in the mesh.
    unsigned int n = mesh.n_vertices();

    typedef Eigen::Triplet<Scalar> Triplet;

    std::vector<Triplet> tripletList;
    tripletList.reserve(n * n);

    // TODO: Fill in the code to compute the graph laplacian matrix.
    // \li First, set the adjacency values correspodning to each vertex in the one-ring.
    // \li Next, compute the degree (valence) as you go along.
    // \li Last, store the degree and adjacency entries in the matrix and generate the laplacian.

    for (auto const& v_i : mesh.vertices())
    {
        for (auto const& edge : mesh.halfedges(v_i))
        {
            auto v_j = mesh.to_vertex(edge);
            tripletList.push_back(Triplet(v_i.idx(), v_j.idx(), 0));
        }

        tripletList.push_back(Triplet(v_i.idx(), v_i.idx(), 1));
    }

    L.setFromTriplets(tripletList.begin(), tripletList.end());
}

void Smoother::smooth_explicit(OpenGP::Scalar lambda)
{
    // Set up our identity matrix.
    int n = mesh.n_vertices();
    SparseMatrix<Scalar> I(n, n);
    I.setIdentity();

    MatrixXf P_t(3, mesh.n_vertices());
    MatrixXf P_t1(3, mesh.n_vertices());

    // Now create the matrix containing all of our points.
    int i = 0;
    for (auto const& vertex : mesh.vertices())
    {
        Point p = mesh.position(vertex);
        P_t.col(i) = p;
        ++i;
    }

    P_t.transposeInPlace();
    P_t1.transposeInPlace();

    // TODO: Fill in the code for explicit smoothing.

    i = 0;
    for (auto const& vertex : mesh.vertices())
    {
        mesh.position(vertex) = P_t.row(i);
        ++i;
    }
}

void Smoother::smooth_implicit(OpenGP::Scalar lambda)
{
    int n = mesh.n_vertices();
    SparseMatrix<Scalar> I(n, n);
    I.setIdentity();

    MatrixXf P_t(3, mesh.n_vertices());
    MatrixXf P_t1(3, mesh.n_vertices());

    int i = 0;
    for (auto const& vertex : mesh.vertices())
    {
        Point p = mesh.position(vertex);
        P_t.col(i) = p;
        ++i;
    }

    P_t.transposeInPlace();
    P_t1.transposeInPlace();

    // TODO: Fill in the code for implicit smoothing.

    i = 0;
    for (auto const& vertex : mesh.vertices())
    {
        mesh.position(vertex) = P_t.row(i);
        ++i;
    }
}

void Smoother::solve_linear_least_square(Eigen::SparseMatrix<OpenGP::Scalar>& A, Eigen::MatrixXf&B, Eigen::MatrixXf& X)
{
    SparseMatrix<Scalar> At = A.transpose();
    SparseMatrix<Scalar> AtA = At * A;

    typedef SimplicialLDLT<SparseMatrix<Scalar>> Solver;
    Solver solver;
    solver.compute(AtA);

    for (int i = 0; i < B.cols(); ++i)
    {
        X.col(i) = solver.solve(At * B.col(i));
    }
}
