#include "Curvature.h"

Curvature::Curvature(OpenGP::SurfaceMesh& mesh) : mesh(mesh) {
    vpoint = mesh.vertex_property<OpenGP::Point>("v:point");
    vquality = mesh.vertex_property<float>("v:quality");
    varea = mesh.add_vertex_property<OpenGP::Scalar>("v:area");
    ecotan = mesh.add_edge_property<OpenGP::Scalar>("e:cotan");
    vcurvature_K = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_K");
    vcurvature_H = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_H");
    vcurvature_k1 = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_k1");
    vcurvature_k2 = mesh.add_vertex_property<OpenGP::Scalar>("v:curvature_k2");
}

Curvature::~Curvature() {
    mesh.remove_vertex_property(varea);
    mesh.remove_edge_property(ecotan);
    mesh.remove_vertex_property(vcurvature_K);
    mesh.remove_vertex_property(vcurvature_H);
    mesh.remove_vertex_property(vcurvature_k1);
    mesh.remove_vertex_property(vcurvature_k2);
}

void Curvature::visualize_gauss_curvature() {
    using namespace OpenGP;
    /**
     *  \todo Compute cotangent weights per edge.
     *  \li for each vertex, find the incident triangle(s) (it can be a boundary edge!)
     *  \li for each incident triangle, collect the three points, compute the angle
     *  opposite of the edge, and add its cotan to the weight of this edge.
     */
    SurfaceMesh::Halfedge nextEdge;
    SurfaceMesh::Vertex v0, v1;
    Point p, p0, p1, d0, d1;
    Scalar theta, area;

    for (auto const& vertex : mesh.vertices())
    {
        theta = 0.0f;
        area = 0.0f;

        for (auto const& edge : mesh.halfedges(vertex))
        {
            nextEdge = mesh.next_halfedge(edge);

            v0 = mesh.from_vertex(nextEdge);
            v1 = mesh.to_vertex(nextEdge);

            p0 = vpoint[v0];
            p1 = vpoint[v1];
            p = vpoint[vertex];

            d0 = p0 - p;
            d1 = p1 - p;
            d0.normalize();
            d1.normalize();

            // Get the angle.
            theta += std::acos(d0.dot(d1));

            // Now get the area.
            area += (1 / 6.0f) * (d0.cross(d1)).norm();
        }

        vcurvature_K[vertex] = (2 * M_PI - theta) / area;
    }

    create_colours(vcurvature_K);
    gaussComputed = true;
}

void Curvature::visualize_mean_curvature() {
    using namespace OpenGP;

    if (!gaussComputed)
        return;

    /**
     *  \todo Compute the Voronoi area for each vertex.
     *  \li Loop over each vertex and set its area to zero.
     *  \li Loop over each triangle and add one third of its area to each of its vertices.
     */
    for (auto const& vertex : mesh.vertices()) {
        varea[vertex] = 0.0;
    }

    SurfaceMesh::Vertex_around_face_circulator vFit;
    SurfaceMesh::Vertex v0, v1, v2;
    Scalar a;

    for (auto const& face : mesh.faces()) {
        // Collect triangle vertices.
        auto vFit = mesh.vertices(face);
        v0 = *vFit;
        ++vFit;
        v1 = *vFit;
        ++vFit;
        v2 = *vFit;

        // Compute one third area.
        a = (vpoint[v1] - vpoint[v0]).cross(vpoint[v2] - vpoint[v0]).norm() / 6.0;

        // Distribute area to vertices
        varea[v0] += a;
        varea[v1] += a;
        varea[v2] += a;
    }

    /**
     *  \todo Compute the mean meanCature for each vertex.
     *  \li For each vertex, loop over its one-ring neighbours with a Halfedge_around_vertex_circulator.
     *  \li Compute the Laplace by summing the vector from the centre vertex to its neighbour
     *  weighted by the cotan edge weight.
     *  \li Compute the mean meanCature from the Laplace vector and store it in the vertex property meanC.
     */
    SurfaceMesh::Halfedge nextEdge;
    Point p, p0, p1, d0, d1;
    Scalar alpha, beta;
    Point laplace;

    for (const auto& vertex : mesh.vertices()) {
        laplace = Point(0, 0, 0);

        if (!mesh.is_boundary(vertex)) {
            for (const auto& neighbour : mesh.halfedges(vertex)) {
                nextEdge = mesh.next_halfedge(neighbour);

                v0 = mesh.from_vertex(nextEdge);
                v1 = mesh.to_vertex(nextEdge);

                p0 = vpoint[v0];
                p1 = vpoint[v1];
                p = vpoint[vertex];

                d0 = p - p0;
                d1 = p1 - p0;

                d0.normalize();
                d1.normalize();

                beta = std::acos(d0.dot(d1));

                d0 = p - p1;
                d1 = p0 - p1;

                d0.normalize();
                d1.normalize();

                alpha = std::acos(d0.dot(d1));

                Scalar cotanAlpha = 1.0f / std::tan(alpha);
                Scalar cotanBeta = 1.0f / std::tan(beta);

                laplace += (cotanAlpha + cotanBeta) * (p0 - p);
            }

            laplace /= 2.0 * varea[vertex];
        }

        vcurvature_H[vertex] = 0.5 * laplace.norm();
    }

    create_colours(vcurvature_H);

    meanComputed = true;
}

void Curvature::visualize_k1_curvature() {
    if (!gaussComputed && !meanComputed)
        return;

    /**
     *  \todo Compute the k1 Principal curvature using the already computed
     *  Gaussian and Mean curvatures.
     */
    for (auto const& vertex : mesh.vertices())
        vcurvature_k1[vertex] = vcurvature_H[vertex] + sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);

    create_colours(vcurvature_k1);
}

void Curvature::visualize_k2_curvature() {
    if (!gaussComputed && !meanComputed)
        return;

    /**
     *  \todo Compute the k2 Principal curvature using the already computed
     *  Gaussian and Mean curvatures.
     */
    for (auto const& vertex : mesh.vertices())
        vcurvature_k2[vertex] = vcurvature_H[vertex] - sqrt((vcurvature_H[vertex] * vcurvature_H[vertex]) - vcurvature_K[vertex]);

    create_colours(vcurvature_k2);
}

void Curvature::create_colours(OpenGP::SurfaceMesh::Vertex_property<OpenGP::Scalar> prop) {
    using namespace OpenGP;

    // Determine min and max meanCature (remove outliers)
    // Use relative property value are 1D texture coordinate in range [0, 1].
    std::vector<Scalar> values;
    for (const auto& vertex : mesh.vertices()) {
        values.push_back(prop[vertex]);
    }

    // Sort array.
    std::sort(values.begin(), values.end());

    // Discard lower and upper 2%.
    unsigned int n = values.size() - 1;
    unsigned int i = n / 50;
    Scalar minProp = values[i];
    Scalar maxProp = values[n-1-i];

    for (const auto& vertex : mesh.vertices()) {
        vquality[vertex] = (prop[vertex] - minProp) / (maxProp - minProp);
    }
}
