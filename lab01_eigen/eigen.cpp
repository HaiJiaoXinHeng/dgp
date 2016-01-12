#include <Eigen/Dense>
#include <iostream>
#include <OpenGP/MLogger.h> /// mDebug() << 
#define UNUSED(expr) (void)expr

int main(){
    // Quick documentation (for MATLAB users)
    // http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    // Quick example: take two vectors, fill them, and do something.
    
    // Another way of initializing a vector/matrix
    

    // Display them to standard output
       

    // Notice that these two now have the same value. Eigen normalizes in place.
    // Note: be careful in performing A=A.something()! See "ALIASING" topic in Eigen
    
    // Now let's set up a matrix.
    
    // Again, inline initialization
    

    // And we can do the operations seen on the slides.
    //auto m1 = m + n; // This will fail.
        
    return 0;
}
