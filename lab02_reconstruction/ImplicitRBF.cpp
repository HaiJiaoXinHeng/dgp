#include "ImplicitRBF.h"
#include <OpenGP/MLogger.h>
#include <OpenGP/SurfaceMesh/Eigen.h>

// A small number for determining the normal offset distance relative to the bounding box diameter
#define OFFSET_EPSILON_R 0.001

/// Solves the linear system Ax=b
void solve_linear_system(const MatMxN& A, const VecN& b, VecN& x){
    // Solve the linear system using Eigen's Householder QR factorization
    Eigen::HouseholderQR<MatMxN> qr(A);
    x = qr.solve(b);
}

void ImplicitRBF::optimize_rbf_weights(){
    mDebug() << "Start Implicit RBF fitting";
    int n = cloud.n_vertices();
    int N = 2 * n;

    // Matrices for the RBF centers and their weights
    centers_.setZero(3, N);
    weights_.setZero(N);

    // Linear system matrix and right-hand-side vector for computing the weights
    MatMxN M(N, N);
    VecN d(N);

    // HOMEWORK TASK:
    // 1) Determine the RBF centers, and store them into the columns of matrix centers_;
    //    The offset distance should be determined according to the bounding box diaginal length.
    // 2) Collect the on- and off-surface constraints, to set up the linear system matrix M
    //    and the right-hand-side d.
    // 3) Use the memeber function solve_linear_system(...) to solve the linear system
    //    to obtain RBF weights, and store them in the data member weights_.
}

Scalar ImplicitRBF::eval_implicit_at(const Vec3& p) const{
    // HOMEWORK TASK: evaluate the RBF implicit function at point "p"
    return p.squaredNorm() - std::pow(.99,2);
}
