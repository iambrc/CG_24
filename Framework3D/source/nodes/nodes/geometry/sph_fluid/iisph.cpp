#include "iisph.h"
#include <iostream>

namespace USTC_CG::node_sph_fluid {

using namespace Eigen;

IISPH::IISPH(const MatrixXd& X, const Vector3d& box_min, const Vector3d& box_max)
    : SPHBase(X, box_min, box_max)
{
    // (HW TODO) Feel free to modify this part to remove or add necessary member variables
    int particle_num = ps_.particles().size();
    predict_density_ = VectorXd::Zero(particle_num);
    aii_ = VectorXd::Zero(particle_num);
    Api_ = MatrixXd::Zero(particle_num, 3);
    last_pressure_ = VectorXd::Zero(particle_num);
    dii_ = MatrixXd::Zero(particle_num, 3);
}

void IISPH::step()
{
    // (HW Optional)
    ps_.assign_particles_to_cells();
    ps_.search_neighbors();
    this->predict_advection();
    this->compute_pressure();
    // Integrate velocity and position
    for (auto& p : ps_.particles()) {
		p->acceleration_ = Vector3d::Zero();
	}
    this->SPHBase::compute_pressure_gradient_acceleration();
    for (auto& p : ps_.particles()) {
        p->vel() += dt_ * p->acceleration_;
        p->x() += dt_ * p->vel();
        check_collision(p);
        vel_.row(p->idx()) = p->vel().transpose();
        X_.row(p->idx()) = p->x().transpose();
    }   
    int particle_num = ps_.particles().size();
    predict_density_ = VectorXd::Zero(particle_num);
    aii_ = VectorXd::Zero(particle_num);
    dii_ = MatrixXd::Zero(particle_num, 3);
    //this->reset();
}

void IISPH::compute_pressure()
{
    // (HW Optional) solve pressure using relaxed Jacobi iteration 
    // Something like this: 
    double threshold = 0.001;
    for (unsigned iter = 0; iter < max_iter_; iter++) {
        double avg_density_error = pressure_solve_iteration();
        //std::cout << avg_density_error << std::endl;
        if (fabs(avg_density_error) < threshold)
            break;
    }
    for (auto& p : ps_.particles()) {
		last_pressure_(p->idx()) = p->pressure();
	}
}

void IISPH::predict_advection()
{
    // (HW Optional)
    // predict new density based on non-pressure forces,
    // compute necessary variables for pressure solve, etc. 

    // Note: feel free to remove or add functions based on your need,
    // you can also rename this function. 
    this->SPHBase::compute_density();
    this->SPHBase::compute_non_pressure_acceleration();
    for (auto& p : ps_.particles()) {
        p->vel() += dt_ * p->acceleration_;
        //vel_.row(p->idx()) = p->vel().transpose();
    }
    for (auto& p : ps_.particles()) {
        for (auto& q : p->neighbors()) {
            dii_.row(p->idx()) -= dt_*dt_*ps_.mass() * grad_W(p->x() - q->x(), ps_.h()) 
                / p->density() / p->density();
        }
	}
    for (auto& p : ps_.particles()) {
        predict_density_(p->idx()) = p->density();
        p->pressure_ = 0.5 * last_pressure_(p->idx());
        for (auto& q : p->neighbors()) {
            predict_density_(p->idx()) +=
                dt_ * ps_.mass() * (p->vel() - q->vel()).dot(grad_W(p->x() - q->x(), ps_.h()));
            Vector3d dji = -dt_*dt_*ps_.mass() * grad_W(q->x() - p->x(), ps_.h()) /
                           p->density() / p->density();
            aii_(p->idx()) += ps_.mass() * grad_W(p->x() - q->x(), ps_.h()).dot(
                dii_.row(p->idx()) - dji.transpose());
        }
    }
}

double IISPH::pressure_solve_iteration()
{
    // (HW Optional)   
    // One step iteration to solve the pressure poisson equation of IISPH
    VectorXd new_pressure = VectorXd::Zero(ps_.particles().size());
    Api_ = MatrixXd::Zero(ps_.particles().size(), 3);
    for (auto& p : ps_.particles()) {
        for (auto& q : p->neighbors()) {
            Api_.row(p->idx()) -= dt_*dt_*ps_.mass() * q->pressure() *
                grad_W(p->x() - q->x(), ps_.h()) / q->density() / q->density();
        }
    }
    double totalsum = 0;
    for (auto& p : ps_.particles()) {
        new_pressure(p->idx()) = (1 - omega_) * p->pressure();
        double sum = 0;
        for (auto& q : p->neighbors()) {
            Vector3d tmp = Api_.row(p->idx()) - Api_.row(q->idx()) - 
                dii_.row(q->idx()) * q->pressure();
            tmp -= dt_*dt_*ps_.mass() * grad_W(q->x() - p->x(), ps_.h()) 
                * p->pressure() / p->density() / p->density();
            sum += ps_.mass() * tmp.dot(grad_W(p->x() - q->x(), ps_.h()));
        }
        if (abs(aii_(p->idx())) > 1e-10)
            new_pressure(p->idx()) +=
                omega_ * ((ps_.density0() - predict_density_(p->idx())) - sum) / aii_(p->idx());
        else
            new_pressure(p->idx()) = 0;
        totalsum += sum + aii_(p->idx()) * p->pressure();
    }
    for (auto& p : ps_.particles()) {
        p->pressure_ = std::clamp(new_pressure(p->idx()), 0.0, 1e5);
	}
    return totalsum / ps_.particles().size();
}

// ------------------ helper function, no need to modify ---------------------
void IISPH::reset()
{
    SPHBase::reset();

    predict_density_ = VectorXd::Zero(ps_.particles().size());
    aii_ = VectorXd::Zero(ps_.particles().size());
    Api_ = MatrixXd::Zero(ps_.particles().size(), 3);
    last_pressure_ = VectorXd::Zero(ps_.particles().size());
    dii_ = MatrixXd::Zero(ps_.particles().size(), 3);
}
}  // namespace USTC_CG::node_sph_fluid