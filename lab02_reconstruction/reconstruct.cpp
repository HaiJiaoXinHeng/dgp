#include <OpenGP/MLogger.h>
#include <OpenGP/SurfaceMesh/bounding_box.h>
#include "Grid.h"
#include "MarchingCubes.h"

#include "ImplicitRBF.h"
#include "ImplicitHoppe.h"

void reconstruct(SurfaceMesh& cloud, SurfaceMesh &output, uint res){    
    // Compute bounding cube for Marching Cubes grid.
    mDebug() << "Computing bounding box\n";
    Box3 bbox = OpenGP::bounding_box(cloud);
    bbox = OpenGP::bbox_cubified(bbox);
    bbox = OpenGP::bbox_scaled(bbox, 1.1);        

    // Setup marching cubes grid.
    std::cout << "Setup regular grid for the signed distance field\n" << std::flush;
    Grid grid(bbox, res, res, res);
    
    // Uncomment the method you would like to employ
    ImplicitHoppe method(cloud);
    // ImplicitRBF method(cloud);
    
    // Get grid's signed distance values by evaluating the implicit function
    for(uint x=0; x < res; ++x)
        for(uint y=0; y < res; ++y)
            for(uint z=0; z < res; ++z)
                grid(x, y, z) = method.eval_implicit_at( grid.point(x, y, z) );
    
    // Extracts isosurface by marching cubes
    std::cout << "Extract isosurface...\n";
    MarchingCubes::exec(grid, output);
    mDebug("Done! [#V:%d #F:%d]", output.n_vertices(), output.n_faces());
}
