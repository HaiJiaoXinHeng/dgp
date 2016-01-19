#include "SurfaceReconstruction.h"
// includes
#include <OpenGP/MLogger.h>
#include <OpenGP/SurfaceMesh/bounding_box.h>
#include "Grid.h"
#include "MarchingCubes.h"

void SurfaceReconstruction::reconstruct(SurfaceMesh &output, uint res){    

    Box3 bbox(Vec3(-1,-1,-1), Vec3(+1,+1,+1));
    // Setup marching cubes grid.
    std::cout << "Setup regular grid for the signed distance field\n" << std::flush;
    Grid grid(bbox, res, res, res);
    
    // build the accelerator data structure
    
    // Get grid's signed distance values by evaluating the implicit function
    for(uint x=0; x < res; ++x)
        for(uint y=0; y < res; ++y)
            for(uint z=0; z < res; ++z)
                grid(x, y, z) = eval_implicit_at( grid.point(x, y, z) );
    
    // Extracts isosurface by marching cubes
    std::cout << "Extract isosurface...\n";
    MarchingCubes::exec(grid, output);
    mDebug("Done! [#V:%d #F:%d]", output.n_vertices(), output.n_faces());
}

Scalar SurfaceReconstruction::eval_implicit_at(const Vec3 &p){
    return p.squaredNorm() - std::pow(.99,2);
}
