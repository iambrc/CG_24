#include "FastMassSpring.h"
#include <iostream>


namespace USTC_CG::node_mass_spring {
FastMassSpring::FastMassSpring(const Eigen::MatrixXd& X, const EdgeSet& E, const float stiffness, const unsigned max_it, const double h_): 
MassSpring(X, E){
    // construct L and J at initialization
    std::cout << "init fast mass spring" << std::endl;

    n_vertices = X.rows();
    n_edges = E.size();
    this->stiffness = stiffness;
    max_iter = max_it;
    h = h_;

    Eigen::SparseMatrix<double> A(n_vertices * 3, n_vertices * 3);
    Eigen::SparseMatrix<double> tmp(n_vertices, n_vertices);
    L.resize(n_vertices * 3, n_vertices * 3);
    L.setZero();
    for (const auto& e : E)
    {
        if (dirichlet_bc_mask[e.first] && !dirichlet_bc_mask[e.second])
        {
            tmp.coeffRef(e.second, e.first) -= 1;
            tmp.coeffRef(e.second, e.second) += 1;
        }
        else if (dirichlet_bc_mask[e.second] && !dirichlet_bc_mask[e.first])
        {
            tmp.coeffRef(e.first, e.first) += 1;
            tmp.coeffRef(e.first, e.second) -= 1;
        }
        else if (!dirichlet_bc_mask[e.first] && !dirichlet_bc_mask[e.second])
        {
            tmp.coeffRef(e.first, e.first) += 1;
            tmp.coeffRef(e.first, e.second) -= 1;
            tmp.coeffRef(e.second, e.first) -= 1;
            tmp.coeffRef(e.second, e.second) += 1;
        }
    }
    for (int i = 0; i < n_vertices; i++)
    {
        for (int j = 0; j < n_vertices; j++)
        {
            L.coeffRef(3 * i, 3 * j) = tmp.coeffRef(i, j);
            L.coeffRef(3 * i + 1, 3 * j + 1) = tmp.coeffRef(i, j);
            L.coeffRef(3 * i + 2, 3 * j + 2) = tmp.coeffRef(i, j);
        }
    }
    
    Eigen::SparseMatrix<double> tmp_(n_vertices, n_edges);
    tmp_.setZero();
    J.resize(n_vertices * 3, n_edges * 3);
    J.setZero();
    unsigned it = 0;
    for (const auto& e : E)
    {
        tmp_.coeffRef(e.first, it) += 1;
        tmp_.coeffRef(e.second, it) -= 1;
        it++;
    }
    std::cout << std::endl;
    for (int i = 0; i < n_vertices; i++) 
    {
        for (int j = 0; j < n_edges; j++) 
        {
            J.coeffRef(3 * i, 3 * j) = tmp_.coeffRef(i, j);
            J.coeffRef(3 * i + 1, 3 * j + 1) = tmp_.coeffRef(i, j);
            J.coeffRef(3 * i + 2, 3 * j + 2) = tmp_.coeffRef(i, j);
        }
    }
    // (HW Optional) precompute A and prefactorize
    J = stiffness * J;
    A = h * h * stiffness * L;
    for (size_t i = 0; i < n_vertices; i++)
    {
        if (dirichlet_bc_mask[i])
        {
            for (size_t j = 0; j < 3; j++)
                A.coeffRef(3 * i + j, 3 * i + j) = 1;
        }
        else
        {
            for (size_t j = 0; j < 3; j++)
                A.coeffRef(3 * i + j, 3 * i + j) += mass / n_vertices;
        }
    }
    A.makeCompressed();
    solver.compute(A);
    // Note: one thing to take care of: A is related with stiffness, 
    // if stiffness changes, A need to be recomputed
}

void FastMassSpring::step()
{
    // (HW Optional) Necessary preparation
    Eigen::VectorXd d(3 * n_edges), b(3 * n_vertices);
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius);
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    acceleration.rowwise() += acceleration_ext.transpose();
    if (enable_sphere_collision) {
        acceleration += acceleration_collision;
    }
    acceleration.row(fix1).setZero();
    acceleration.row(fix2).setZero();
    Eigen::MatrixXd X_prev = X;
    Eigen::MatrixXd Y = X + h * vel + h * h * acceleration;
    Y = flatten(Y);
    for (unsigned iter = 0; iter < max_iter; iter++) {
        // (HW Optional)
        // local_step and global_step alternating solving

        // local
        unsigned it = 0;
        for (const auto& e : E) {
            auto xi = X.row(e.first) - X.row(e.second);
            double norm = xi.norm();
            double l = E_rest_length[it];
            d(3 * it) = l * xi(0) / norm;
            d(3 * it + 1) = l * xi(1) / norm;
            d(3 * it + 2) = l * xi(2) / norm;
            it++;
        }
        // global
        b = h * h * (J * d) + Y * (mass / n_vertices);
        Eigen::Vector3d fix_pos1, fix_pos2;
        for (size_t i = 0; i < 3; i++)
        {
            fix_pos1(i) = X.coeffRef(fix1, i);
            fix_pos2(i) = X.coeffRef(fix2, i);
        }
        for (size_t i = 0; i < 3; i++)
        {
            b(3 * fix1 + i) = fix_pos1(i);
            b(3 * fix2 + i) = fix_pos2(i);
        }
        X = unflatten(solver.solve(b));
        //std::cout << vel.row(2) << std::endl;
    }
    vel = (X - X_prev) / h;
    //vel.row(fix1).setZero();
    //vel.row(fix2).setZero();
    if (enable_damping)
        vel *= damping;
}

}  // namespace USTC_CG::node_mass_spring
