#include <Eigen/Dense>
#include <iostream>
#include <OpenGP/MLogger.h> /// mDebug() << 
#define UNUSED(expr) (void)expr

int main(){
    // Quick documentation (for MATLAB users)
    // http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    // Quick example: take two vectors, fill them, and do something.
    Eigen::Vector3f u, v;
    u(0) = 1.0;
    u(1) = 0.0;
    u(2) = 0.0;

    // Another way of initializing a vector/matrix
    v << 0.0, 1.0, 0.0;

    auto wStar = u + v;

    mDebug() << "wStar: " << wStar.transpose();

    // Display them to standard output
    mDebug() << "u: " << u.transpose();
    mDebug() << "v: " << v.transpose();

    auto w = u.cross(v);
    mDebug() << u.transpose() << " dot " << v.transpose() << " = " << w.transpose();
    double dot = u.dot(v);
    mDebug() << u.transpose() << " dot " << v.transpose() << " = " << dot;
    

    // Notice that these two now have the same value. Eigen normalizes in place.
    // Note: be careful in performing A=A.something()! See "ALIASING" topic in Eigen
    auto uHat = u.normalized();
    u.normalize();
    auto normU = u.norm();
    auto normUHat = uHat.norm();
    mDebug() << "\n";
    mDebug() << "normU:" << normU;
    mDebug() << "normUHat: " << normUHat;

    // Now let's set up a matrix.
    Eigen::Matrix2f m;
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << "\nMatrix m:\n" << m << std::endl; 

    // Again, inline initialization
    Eigen::Matrix3f n;
    n << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    std::cout << "\nMatrix n:\n" << n << std::endl; 

    // And we can do the operations seen on the slides.
    //auto m1 = m + n; // This will fail.
    auto m2 = Eigen::Matrix2f::Identity();
    auto m3 = m + m2; // This will not.
    std::cout << "\nMatrix m3:\n" << m3 << std::endl; 
    
    return 0;
}
