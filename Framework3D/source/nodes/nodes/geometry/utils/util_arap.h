#pragma once

#include "util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>

namespace USTC_CG
{
	class Arap
	{
		public:
            Arap() = default;

			~Arap() = default;

            void init(std::shared_ptr<PolyMesh> mesh1, std::shared_ptr<PolyMesh> mesh2);
			// calculate cotangent value of each halfedge's oppsite angle
			void set_cotangent();
			// initialize the matrix A
			void set_matrixA();
            // set two fixed point's index
            void set_fixed(int a, int b);
			// set initial uv
            void set_uv_mesh();
			// set flat triangle
            void set_flatxy();
			// calculate the initial matrix of each face
			void set_matrixLt();
			// apply SVD decompose to each face's matrix
            void SVD_Lt();
			// set the bx, by & calculate new_x, new_y
            void set_Laplacian();
			// calculate energy
            double energy_cal();
			// set the new mesh
            void set_new_mesh();
			// loop, reset mesh
            void reset_mesh();
			// get result
            pxr::VtArray<pxr::GfVec2f> get_new_mesh();

		private:
            std::shared_ptr<PolyMesh> origin_mesh, mesh;
            pxr::VtArray<double> cotangent;
            Eigen::SparseMatrix<double> A;
            Eigen::VectorXd bx, by, new_x, new_y;
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            pxr::VtArray<Eigen::Matrix2d> Lt;
            pxr::VtArray<Eigen::Matrix2d> SVDLt;
            pxr::VtArray<pxr::GfVec2f> uv_mesh, uv_result;
            pxr::VtArray<OpenMesh::Vec2f> flatxy;
            double energy;
            int fix_ind1, fix_ind2;
	};
}