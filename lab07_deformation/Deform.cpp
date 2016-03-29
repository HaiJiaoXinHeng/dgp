#include "Deform.h"

#include <OpenGP/MLogger.h>
#include <OpenGP/SurfaceMesh/eigen.h>

#include "Laplacian.h"

using namespace OpenGP;

#if QUAD_MESH
const float handleRadius = 0.0f;

const std::vector<int> handleEntries = { 0, 1, 2 };
const std::vector<int> handleIds = { 1, 2, 3};
#else
const float handleRadius = 0.2f;

const std::vector<int> handleEntries = { 8, 22, 73, 90, 183 };
const std::vector<int> handleIds = { 1, 2, 3, 4, 5 };
#endif

Eigen::SimplicialCholesky<Eigen::SparseMatrix<OpenGP::Scalar>,
    Eigen::RowMajor> solver;

Deform::Deform(OpenGP::SurfaceMesh& mesh, OpenGP::SceneGraph& scene) :
    mesh(mesh), scene(scene)
{
    vpoint = mesh.vertex_property<Point>("v:point");
    vhandle = mesh.add_vertex_property<int>("v:handle");

    for (auto const& vertex : mesh.vertices())
    {
        vhandle[vertex] = 0;
    }

    selected_handle = 0;
}

Deform::~Deform()
{
    mesh.remove_vertex_property(vhandle);
}

void Deform::construct_handles()
{
    int i = 0;
    k = 0;
    for (const auto& vertex : mesh.vertices())
    {
        // If we have selected all of the handles, then exit the loop.
        if (i == handleEntries.size())
        {
            break;
        }
        
        // Check if the vertex is a handle.
        if (vertex.idx() == handleEntries[i])
        {
            // Set the handle first.
            vhandle[vertex] = handleIds[i];
            ++k;

            // Grab the current position of the vertex.
            auto start = mesh.position(vertex);

            // Now loop over all the vertices again and find those
            // that are within handleRadius of start.
            for (auto const& neighbour : mesh.vertices())
            {
                if (neighbour == vertex)
                {
                    continue;
                }

                auto member = mesh.position(neighbour);

                float distance = (member - start).norm();

                if (distance < handleRadius)
                {
                    vhandle[neighbour] = handleIds[i];
                    ++k;
                }

            }
            ++i;
        }
    }

    // At this stage we know how many handles there are, so we can compute
    // u and k so we can access them later.
    u = mesh.n_vertices() - k;
}

void Deform::intialize_laplacian()
{
    factor_matrices();
}

void Deform::select_handle(int index)
{
    // We don't accept handle 0 or anything beyond the size.
    if (index < 1 || index > handleEntries.size())
    {
        return;
    }

    // Store the selected handle.
    selected_handle = index;

    // Smooth the mesh with the initial positions of the handles.
    smooth_mesh();

    // Compute displacements.
    compute_displacements();
    
    // Update the mesh so we can see the smoothed version.
    //update_mesh();
}

void Deform::mouse_down(OpenGP::Point const& pos)
{
    if (selected_handle == 0)
    {
        return;
    }

    /**
     * \todo Move the selected handle to the current mouse position based on
     * the barycentre (geometric mean) of the handle.
     * \li First, obtain all the points that belong to the selected handle.
     * \li Next, compute the barycentre (geometric mean) of the set.
     * \li Compute the displacement between the barycentre and the 
     * cursor position.
     * \li Translate all the points in set using the displacement.
     * \li Update the mesh.
     */
}

void Deform::mouse_up()
{
    if (selected_handle == 0)
    {
        return;
    }

    update_mesh_with_displacements();
}

Eigen::SparseMatrix<OpenGP::Scalar> Deform::construct_squared_laplacian()
{
    auto L = Laplacian::graph_laplacian(mesh);
    return L * L;
}

void Deform::construct_permutation_matrix()
{
    /**
     * \todo Construct the permutation matrix that splits the point matrix V 
     * and the Laplacian matrix L into the required blocks.
     * \li First, store the indices of all the vertex handles (remember, a
     * vertex that isn't a handle has vhandle set to 0).
     * \li Next, identify those indices that will remain fixed by the
     * permutation.
     * \li Permute the indices. Remember: in the case of the V matrix, the 
     * final result is the following:
     *
     *           u             k
     * V = (p p p .... p | h h ... h)
     */
}


void Deform::factor_matrices()
{
    /**
     * \todo Perform the matrix factorization that allows us to deform in real
     * time.
     * \li Get the squared laplacian.
     * \li Construct the permutation matrix.
     * \li Permute V (the points matrix, see vertices_matrix), and the
     * Laplacian.
     * \li Assign L_uu, L_uk, v_u, and v_k to the appropriate blocks of the 
     * corresponding matrices (see block).
     * \li Set the handle renderer to visualize your handles.
     * \li Compute the Cholesky decomposition of L_uu.
     */
}

void Deform::smooth_mesh()
{
    // Compute the B matrix first.
    Eigen::MatrixXf B(L_uk.rows(), 3);
    B = -1.0f * L_uk * v_k.transpose();

    // Now solve the system.
    v_u.transposeInPlace();
    for (int i = 0; i < B.cols(); ++i)
    {
        // We operate on v_u transpose, so we use the rows of v_u instead
        // of the columns.
        v_u.col(i) = solver.solve(B.col(i));
    }

    v_u.transposeInPlace();
}

void Deform::compute_displacements()
{
    /**
     * \todo Compute the displacements between the original (un-smoothed)
     * points and the smoothed points.
     * \li Obtain the vertex matrix and permute it.
     * \li Now for each column of the v_u region, compute the displacement 
     * between it and v_u.
     * \li Don't forget to permute V to restore it!
     */
}

void Deform::update_mesh()
{
    handleRenderer.init_data(v_k);
    // Re-construct the V matrix.
    Eigen::MatrixXf V(3, mesh.n_vertices());
    V << v_u, v_k;

    // Now unshuffle the entries in the matrix.
    V = V * permute.transpose();

    // Set them back into the point matrix.
    vertices_matrix(mesh) = V;
}

void Deform::update_mesh_with_displacements()
{
    /**
     * \todo Update the mesh with the modified v_u and v_k taking into account
     * the displacements we computed earlier.
     * \li Construct V from v_u and v_k.
     * \li Add the displacements.
     * \li Permute V to restore it to its original state.
     * \li Set it back to the mesh.
     */
}
