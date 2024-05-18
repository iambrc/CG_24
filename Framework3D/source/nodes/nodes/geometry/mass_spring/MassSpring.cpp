#include "MassSpring.h"
#include <iostream>

namespace USTC_CG::node_mass_spring {
MassSpring::MassSpring(const Eigen::MatrixXd& X, const EdgeSet& E)
{
    this->X = this->init_X = X;
    this->vel = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    this->E = E;

    std::cout << "number of edges: " << E.size() << std::endl;
    std::cout << "init mass spring" << std::endl;

    // Compute the rest pose edge length
    for (const auto& e : E) {
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        this->E_rest_length.push_back((x0 - x1).norm());
    }
    

    // Initialize the mask for Dirichlet boundary condition
    dirichlet_bc_mask.resize(X.rows(), false);

    // (HW_TODO) Fix two vertices, feel free to modify this 
    unsigned n_fix = sqrt(X.rows());  // Here we assume the cloth is square
    dirichlet_bc_mask[0] = true;
    dirichlet_bc_mask[n_fix - 1] = true;
    fix1 = 0;
    fix2 = n_fix - 1;
}

void MassSpring::step()
{
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;

    unsigned n_vertices = X.rows();

    // The reason to not use 1.0 as mass per vertex: the cloth gets heavier as we increase the resolution
    double mass_per_vertex =
        mass / n_vertices; 

    //----------------------------------------------------
    // (HW Optional) Bonus part: Sphere collision
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius);
    //----------------------------------------------------

    if (time_integrator == IMPLICIT_EULER) {
        // Implicit Euler
        TIC(step)

        // (HW TODO) 
        auto H_elastic = computeHessianSparse(stiffness);  // size = [nx3, nx3]
        
        // compute Y 
        Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(X.rows(), X.cols());
        acceleration.rowwise() += acceleration_ext.transpose();
        auto Y = X + h * vel + h * h * acceleration;

        // compute grad g
        double err = 1;
        int step_ = 1;
        while (err > 1e-3 && step_ <= 15)
        {
            auto grad_g = flatten(mass_per_vertex * (X - Y) / (h * h) + computeGrad(stiffness));
            for (size_t i = 0; i < n_vertices; i++) {
                if (dirichlet_bc_mask[i]) {
                    grad_g(3 * i, 0) = 0;
                    grad_g(3 * i + 1, 0) = 0;
                    grad_g(3 * i + 2, 0) = 0;
                }
            }
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(H_elastic);
            auto delta_x = unflatten(solver.solve(grad_g));

            X -= delta_x;
            err = grad_g.norm();
            step_++;
        }

        // Solve Newton's search direction with linear solver 
        
        //std::cout << delta_x;

        // update X and vel 
        
        acceleration -= computeGrad(stiffness) / mass_per_vertex;
        if (enable_sphere_collision) {
            acceleration += acceleration_collision;
        }
        vel += h * acceleration;
        if (enable_damping)
            vel *= damping;

        TOC(step)
    }
    else if (time_integrator == SEMI_IMPLICIT_EULER) {

        // Semi-implicit Euler
        Eigen::MatrixXd acceleration = -computeGrad(stiffness) / mass_per_vertex;
        acceleration.rowwise() += acceleration_ext.transpose();

        // -----------------------------------------------
        // (HW Optional)
        if (enable_sphere_collision) {
            acceleration += acceleration_collision;
        }
        // -----------------------------------------------

        // (HW TODO): Implement semi-implicit Euler time integration
        
        // Update X and vel 
        vel += h * acceleration;
        for (size_t i = 0; i < vel.rows(); i++) {
            if (dirichlet_bc_mask[i])
                vel.row(i).setZero();
        }
        if (enable_damping)
            vel *= damping;
        X += h * vel;
        
    }
    else {
        std::cerr << "Unknown time integrator!" << std::endl;
        return;
    }
}

// There are different types of mass spring energy:
// For this homework we will adopt Prof. Huamin Wang's energy definition introduced in GAMES103
// course Lecture 2 E = 0.5 * stiffness * sum_{i=1}^{n} (||x_i - x_j|| - l)^2 There exist other
// types of energy definition, e.g., Prof. Minchen Li's energy definition
// https://www.cs.cmu.edu/~15769-f23/lec/3_Mass_Spring_Systems.pdf
double MassSpring::computeEnergy(double stiffness)
{
    double sum = 0.;
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        sum += 0.5 * stiffness * std::pow((diff.norm() - l), 2);
        i++;
    }
    return sum;
}

Eigen::MatrixXd MassSpring::computeGrad(double stiffness)
{
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    unsigned i = 0;
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the gradient computation
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        auto grad_e = stiffness * (diff.norm() - l) * diff / diff.norm();
        for (size_t i = 0; i < 3; i++)
        {
            g.coeffRef(e.first, i) += grad_e(i);
            g.coeffRef(e.second, i) -= grad_e(i);
        }
        // --------------------------------------------------
        i++;
    }
    //std::cout << g.row(2) << std::endl;
    return g;
}

Eigen::SparseMatrix<double> MassSpring::computeHessianSparse(double stiffness)
{
    unsigned n_vertices = X.rows();
    Eigen::SparseMatrix<double> H(n_vertices * 3, n_vertices * 3);

    unsigned i = 0;
    auto k = stiffness;
    const auto I = Eigen::MatrixXd::Identity(3, 3);
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the sparse version Hessian computation
        // Remember to consider fixed points 
        // You can also consider positive definiteness here
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        Eigen::MatrixXd He = Eigen::MatrixXd::Zero(3, 3);
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                He(i, j) = diff(i) * diff(j);
        He = He / (diff.norm() * diff.norm());

        if (l > diff.norm())
            He = stiffness * He;
        else
            He = stiffness * He + stiffness * (1 - l / diff.norm()) * (I - He);

        if (dirichlet_bc_mask[e.first])
        {
            for (size_t i = 0; i < 3; i++)
                H.coeffRef(3 * e.first + i, 3 * e.first + i) = 1;
            for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                    H.coeffRef(3 * e.second + i, 3 * e.second + j) += He(i, j);
        }  
        else if (dirichlet_bc_mask[e.second])
        {
            for (size_t i = 0; i < 3; i++)
                H.coeffRef(3 * e.second + i, 3 * e.second + i) = 1;
            for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                    H.coeffRef(3 * e.first + i, 3 * e.first + j) += He(i, j);
        } 
        else
        {
            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    H.coeffRef(3 * e.first + i, 3 * e.first + j) += He(i, j);
                    H.coeffRef(3 * e.second + i, 3 * e.second + j) += He(i, j);
                    H.coeffRef(3 * e.first + i, 3 * e.second + j) = -He(i, j);
                    H.coeffRef(3 * e.second + i, 3 * e.first + j) = -He(i, j);
                }
            }
        }
        for (size_t i = 0; i < 3 * n_vertices; i++)
        {
            H.coeffRef(i, i) += mass / n_vertices / h / h;
        }
        // --------------------------------------------------

        i++;
    }
    
    H.makeCompressed();
    return H;
}


bool MassSpring::checkSPD(const Eigen::SparseMatrix<double>& A)
{
    // Eigen::SimplicialLDLT<SparseMatrix_d> ldlt(A);
    // return ldlt.info() == Eigen::Success;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
    auto eigen_values = es.eigenvalues();
    return eigen_values.minCoeff() >= 1e-10;
}

void MassSpring::reset()
{
    std::cout << "reset" << std::endl;
    this->X = this->init_X;
    this->vel.setZero();
}

// ----------------------------------------------------------------------------------
// (HW Optional) Bonus part
Eigen::MatrixXd MassSpring::getSphereCollisionForce(Eigen::Vector3d center, double radius)
{
    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    for (int i = 0; i < X.rows(); i++) {
       // (HW Optional) Implement penalty-based force here 
        auto dir = Vector3d(X.row(i)) - center;
        auto tmp = collision_scale_factor * radius / dir.norm() - 1;
        if (tmp < 0)
            tmp = 0;
        force.row(i) = collision_penalty_k * tmp * dir;
    }
    return force;
}
// ----------------------------------------------------------------------------------


}  // namespace USTC_CG::node_mass_spring

