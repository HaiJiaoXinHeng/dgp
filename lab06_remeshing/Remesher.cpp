#include "Remesher.h"

using namespace Eigen;
using namespace OpenGP;

Remesher::Remesher(OpenGP::SurfaceMesh& m) :
    mesh(m)
{
    points = mesh.vertex_property<Point>("v:point");
    vnormals = mesh.vertex_property<Normal>("v:normal");
}

void Remesher::remesh(OpenGP::Scalar edge_length)
{
    target_edge_length = edge_length;

    // Perform one iteration of remeshing.
    split_long_edges();
    collapse_short_edges();
    flip_edges();
    tangential_smoothing();
}

void Remesher::split_long_edges()
{
    /**
     *	\todo Split edges that are too long.
     *	\li Loop several times over all edges.
     *	\li Determine whether the edge is too long using is_to_long().
     *	\li If an edge is too long, split it (see SurfaceMesh::split(edge, vertex),
     *	set the position of the new vertex to the edge midpoint, and set its normal
     *	to the average of the endpoint normals.
     */

    SurfaceMesh::Vertex v, v0, v1;
    bool done;
    int i;

    // do at most 10 loops over all edges.
    for (done = false, i = 0; !done && i < 10; ++i)
    {
        done = true;

        for (auto const& edge : mesh.edges())
        {
            v0 = mesh.vertex(edge, 0);
            v1 = mesh.vertex(edge, 1);

            if (is_too_long(v0, v1))
            {
                v = mesh.add_vertex((points[v0] + points[v1]) * 0.5f);
                vnormals[v] = (vnormals[v0] + vnormals[v1]).normalized();
                mesh.split(edge, v);
                done = false;
            }
        }
    }
}

void Remesher::collapse_short_edges()
{
    /**
     *	\todo Collapse edges that are too short.
     *	\li Loop several times over all edges.
     *	\li Determine if the edge is too short using is_too_short().
     *	\li If the edge is to short, collapse it (see SurfaceMesh::collapse(halfedge).
     *	\li Get rid of any extraenous edges (see SurfaceMesh::garbage_collection()).
     */

    SurfaceMesh::Vertex v0, v1;
    SurfaceMesh::Halfedge h;
    bool done;
    int i;

    for (done = false, i = 0; !done && i < 10; ++i)
    {
        done = true;

        for (auto const& edge : mesh.edges())
        {
            h = mesh.halfedge(edge, 0);
            v0 = mesh.from_vertex(h);
            v1 = mesh.to_vertex(h);

            // edge too short -> we want to collapse.
            if (is_too_short(v0, v1))
            {
                // is collapse legal?
                if (mesh.is_collapse_ok(h))
                {
                    bool ok = true;
                    for (auto const& v : mesh.vertices(v0))
                    {
                        if (is_too_long(v, v1))
                        {
                            ok = false;
                            break;
                        }
                    }

                    if (ok)
                    {
                        mesh.collapse(h);
                        done = false;
                    }
                }
            }
        }
    }
    mesh.garbage_collection();
}

void Remesher::flip_edges()
{
    /**
     *	\todo Flip edges to improve vertex valences.
     *	\li Loop several times over all edges (only consider non-boundary edges)
     *	\li Collect the four vertices of the two incident triangles.
     *	\li Determine the optimal valences for these vertices (boundary or non-boundary vertex?)
     *	\li If an edge flip improves the valences, then we flip it.
     *	\li Compare the sum of squared deviations from the optimal values.
     */

    SurfaceMesh::Vertex v0, v1, v2, v3;
    SurfaceMesh::Halfedge h;
    int val0, val1, val2, val3;
    int val_opt0, val_opt1, val_opt2, val_opt3;
    int ve_before, ve_after;
    bool done;
    int i;

    for (done = false, i = 0; !done && i < 10; ++i)
    {
        done = true;

        for (auto const& edge : mesh.edges())
        {
            if (!mesh.is_boundary(edge))
            {
                h = mesh.halfedge(edge, 0);
                v0 = mesh.to_vertex(h);
                v2 = mesh.to_vertex(mesh.next_halfedge(h));
                h = mesh.halfedge(edge, 1);
                v1 = mesh.to_vertex(h);
                v3 = mesh.to_vertex(mesh.next_halfedge(h));

                val0 = mesh.valence(v0);
                val1 = mesh.valence(v1);
                val2 = mesh.valence(v2);
                val3 = mesh.valence(v3);

                val_opt0 = (mesh.is_boundary(v0) ? 4 : 6);
                val_opt1 = (mesh.is_boundary(v1) ? 4 : 6);
                val_opt2 = (mesh.is_boundary(v2) ? 4 : 6);
                val_opt3 = (mesh.is_boundary(v3) ? 4 : 6);

                ve_before = (
                    pow(val0 - val_opt0, 2.0f) +
                    pow(val1 - val_opt1, 2.0f) +
                    pow(val2 - val_opt2, 2.0f) +
                    pow(val3 - val_opt3, 2.0f));

                --val0;
                --val1;
                ++val2;
                ++val3;

                ve_after = (
                    pow(val0 - val_opt0, 2.0f) +
                    pow(val1 - val_opt1, 2.0f) +
                    pow(val2 - val_opt2, 2.0f) +
                    pow(val3 - val_opt3, 2.0f));

                if (ve_before > ve_after && mesh.is_flip_ok(edge))
                {
                    mesh.flip(edge);
                    done = false;
                }
            }
        }

    }
}

void Remesher::tangential_smoothing()
{
    /**
     *	\todo Tangential smoothing to improve vertex distribution.
     *	\li Loop 10 times over all vertices, only consider non-boundary vertices.
     *	\li Compute uniform laplacian: vector from centre vertex to barycenter of its one-ring neighbours.
     *	\li Project this vector onto the tangent plane by subtracting its component in normal direction.
     */

    Scalar valence;
    Point u, n;

    for (unsigned int iters = 0; iters < 10; ++iters)
    {
        // Compute tangential updates.
        for (auto const& vertex : mesh.vertices())
        {
            if (!mesh.is_boundary(vertex))
            {
                // Compute barycenter of neighbours.
                u = Point(0.0f, 0.0f, 0.0f);
                valence = 0;
                for (auto const& neighbour : mesh.vertices(vertex))
                {
                    u += points[neighbour];
                    ++valence;
                }

                u *= (1.0f / valence);

                // Project vector from point to barycenter to tangent plane.
                u -= points[vertex];
                u -= vnormals[vertex] * u.dot(vnormals[vertex]);

                // Move point.
                points[vertex] += u;
            }
        }

        mesh.update_vertex_normals();
    }
}
