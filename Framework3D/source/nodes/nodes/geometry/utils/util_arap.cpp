#include "util_arap.h"

namespace USTC_CG
{
void USTC_CG::Arap::init(std::shared_ptr<PolyMesh> mesh1, std::shared_ptr<PolyMesh> mesh2)
{
    origin_mesh = mesh1;
    mesh = mesh2;
}

void USTC_CG::Arap::set_cotangent()
{
    cotangent.resize(origin_mesh->n_halfedges());

    for (const auto& halfedge_handle : origin_mesh->halfedges()) {
        if (halfedge_handle.is_boundary()) {
            cotangent[halfedge_handle.idx()] = 0;
        }
        else {
            const auto& v1 = halfedge_handle.prev().from();
            const auto& v2 = halfedge_handle.from();
            const auto& v3 = halfedge_handle.to();
            const auto& vec1 = origin_mesh->point(v1) - origin_mesh->point(v2);
            const auto& vec2 = origin_mesh->point(v1) - origin_mesh->point(v3);
            double cos_ = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
            cotangent[halfedge_handle.idx()] = 1.0 / tan(acos(cos_));
        }
    }
}

void USTC_CG::Arap::set_matrixA()
{
    size_t size = origin_mesh->n_vertices();
    A.resize(size, size);
    typedef Eigen::Triplet<double> T;
    pxr::VtArray<T> tlist;

    for (const auto& vertex_handle : origin_mesh->vertices()) 
    {
        if (vertex_handle.idx() == fix_ind1 || vertex_handle.idx() == fix_ind2)
        {
            tlist.push_back(T(vertex_handle.idx(), vertex_handle.idx(), 1));
        }
        else
        {
            double tmp = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) 
            {
                double tmp_ = cotangent[halfedge_handle.idx()] + 
                cotangent[halfedge_handle.opp().idx()];
                tmp += tmp_;
                if (halfedge_handle.to().idx() != fix_ind1 && halfedge_handle.to().idx() != fix_ind2)
                    tlist.push_back(T(vertex_handle.idx(), halfedge_handle.to().idx(), -tmp_));
            }
            tlist.push_back(T(vertex_handle.idx(), vertex_handle.idx(), tmp));
        }
    }
    A.setFromTriplets(tlist.begin(), tlist.end());
    solver.compute(A);
}

void USTC_CG::Arap::set_fixed(int a, int b)
{
    fix_ind1 = a;
    fix_ind2 = b;
}

void USTC_CG::Arap::set_uv_mesh()
{
    uv_mesh.resize(mesh->n_vertices());
    for (const auto& vertex_handle : mesh->vertices())
    {
        const auto& vec = mesh->point(vertex_handle);
        uv_mesh[vertex_handle.idx()][0] = vec[1];
        uv_mesh[vertex_handle.idx()][1] = vec[2];
    }
}

void USTC_CG::Arap::set_flatxy()
{
    flatxy.resize(mesh->n_halfedges());
    // caution reverse triangle!!!
    for (const auto& face_handle : origin_mesh->faces())
    {
        int indx = face_handle.idx();
        pxr::VtArray<OpenMesh::Vec3f> xy;
        const auto& halfedge_handle = face_handle.halfedge();
        int ind0 = halfedge_handle.next().to().idx();
        int ind1 = halfedge_handle.from().idx();
        int ind2 = halfedge_handle.to().idx();
        xy.push_back(origin_mesh->point(halfedge_handle.next().to()));
        xy.push_back(origin_mesh->point(halfedge_handle.from()));
        xy.push_back(origin_mesh->point(halfedge_handle.to()));
        flatxy[halfedge_handle.idx()][0] = 0, flatxy[halfedge_handle.idx()][1] = 0;
        flatxy[halfedge_handle.next().idx()][0] = (xy[0] - xy[1]).norm(),
        flatxy[halfedge_handle.next().idx()][1] = 0;
        double cos_ =
            (xy[0] - xy[1]).dot(xy[0] - xy[2]) / ((xy[0] - xy[1]).norm() * (xy[0] - xy[2]).norm());
        flatxy[halfedge_handle.prev().idx()][0] = (xy[0] - xy[2]).norm() * cos_;
        if ((uv_mesh[ind0][0] - uv_mesh[ind1][0]) * (uv_mesh[ind0][1] - uv_mesh[ind2][1]) -
            (uv_mesh[ind0][0] - uv_mesh[ind2][0]) * (uv_mesh[ind0][1] - uv_mesh[ind1][1]) >= 0)
            flatxy[halfedge_handle.prev().idx()][1] = (xy[0] - xy[2]).norm() * sin(acos(cos_));
        else
            flatxy[halfedge_handle.prev().idx()][1] = -(xy[0] - xy[2]).norm() * sin(acos(cos_));
    }
}

void USTC_CG::Arap::set_matrixLt()
{
    Lt.resize(origin_mesh->n_faces());
    for (const auto& face_handle : origin_mesh->faces()) {
        Eigen::Matrix2d Tmp;
        Tmp.setZero();
        int indx = face_handle.idx();
        for (const auto& halfedge_handle : face_handle.halfedges())
        {
            int indh = halfedge_handle.idx(), indhn = halfedge_handle.next().idx(), 
                indhp = halfedge_handle.prev().idx();
            int indi = halfedge_handle.from().idx();
            int indj = halfedge_handle.to().idx();
            Tmp(0, 0) += cotangent[indh] * (uv_mesh[indi][0] - uv_mesh[indj][0]) *
                         (flatxy[indhn][0] - flatxy[indhp][0]);
            Tmp(0, 1) += cotangent[indh] * (uv_mesh[indi][0] - uv_mesh[indj][0]) *
                         (flatxy[indhn][1] - flatxy[indhp][1]);
            Tmp(1, 0) += cotangent[indh] * (uv_mesh[indi][1] - uv_mesh[indj][1]) *
                         (flatxy[indhn][0] - flatxy[indhp][0]);
            Tmp(1, 1) += cotangent[indh] * (uv_mesh[indi][1] - uv_mesh[indj][1]) *
                         (flatxy[indhn][1] - flatxy[indhp][1]);
        }
        Lt[indx] = Tmp;
    }
}

void USTC_CG::Arap::SVD_Lt()
{
    SVDLt.resize(Lt.size());
    for (int i = 0; i < Lt.size(); i++)
    {
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(Lt[i], Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU(), V = svd.matrixV();
        SVDLt[i] = U * V.transpose();
        // signed version!!!
        if (SVDLt[i].determinant() < 0)
        {
            U(0, 1) = -U(0, 1);
            U(1, 1) = -U(1, 1);
            SVDLt[i] = U * V.transpose();
        }
    }
}

void USTC_CG::Arap::set_Laplacian()
{
    int size = mesh->n_vertices();
    bx.resize(size);
    by.resize(size);
    new_x.resize(size);
    new_y.resize(size);
    for (const auto& vertex_handle : mesh->vertices())
    {
        int indx = vertex_handle.idx();
        if (indx == fix_ind1 || indx == fix_ind2)
        {
            bx(indx) = uv_mesh[indx][0];
            by(indx) = uv_mesh[indx][1];
        }
        else
        {
            bx(indx) = 0;
            by(indx) = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
                int indhp = halfedge_handle.prev().idx(), indhn = halfedge_handle.next().idx();
                OpenMesh::Vec2f vec;
                vec[0] = flatxy[indhn][0] - flatxy[indhp][0];
                vec[1] = flatxy[indhn][1] - flatxy[indhp][1];
                Eigen::Matrix2d tmpij;
                Eigen::Matrix2d tmpji;
                tmpji << 0, 0, 0, 0;
                tmpij << 0, 0, 0, 0;
                if (!halfedge_handle.opp().is_boundary())
                    tmpji = SVDLt[halfedge_handle.opp().face().idx()];
                if (!halfedge_handle.is_boundary())
                    tmpij = SVDLt[halfedge_handle.face().idx()];
                int indxij = halfedge_handle.idx();
                int indxji = halfedge_handle.opp().idx();
                bx(indx) +=
                    (cotangent[indxij] * tmpij(0, 0)) * vec[0] +
                    (cotangent[indxij] * tmpij(0, 1)) * vec[1];
                by(indx) +=
                    (cotangent[indxij] * tmpij(1, 0)) * vec[0] +
                    (cotangent[indxij] * tmpij(1, 1)) * vec[1];

                
                int indhp_ = halfedge_handle.opp().prev().idx(), indhn_ = halfedge_handle.opp().next().idx();
                OpenMesh::Vec2f vec_;
                vec_[0] = flatxy[indhp_][0] - flatxy[indhn_][0];
                vec_[1] = flatxy[indhp_][1] - flatxy[indhn_][1];
                bx(indx) += (cotangent[indxji] * tmpji(0, 0)) * vec_[0] +
                            (cotangent[indxji] * tmpji(0, 1)) * vec_[1];
                by(indx) += (cotangent[indxji] * tmpji(1, 0)) * vec_[0] +
                            (cotangent[indxji] * tmpji(1, 1)) * vec_[1];
                
                int indv = halfedge_handle.to().idx();
                if (indv == fix_ind1 || indv == fix_ind2)
                {
                    bx(indx) += (cotangent[indxij] + cotangent[indxji]) * uv_mesh[indv][0];
                    by(indx) += (cotangent[indxij] + cotangent[indxji]) * uv_mesh[indv][1];
                }
            }
        }
    }
    new_x = solver.solve(bx);
    new_y = solver.solve(by);
}

double USTC_CG::Arap::energy_cal()
{
    energy = 0;
    for (const auto& halfedge_handle : mesh->halfedges())
    {
        if (!halfedge_handle.is_boundary())
        {
            int indL = halfedge_handle.face().idx();
            int indi = halfedge_handle.from().idx(), indj = halfedge_handle.to().idx();
            int inde = halfedge_handle.idx();
            int indhp = halfedge_handle.prev().idx(), indhn = halfedge_handle.next().idx();
            OpenMesh::Vec2f vec;
            vec[0] = flatxy[indhn][0] - flatxy[indhp][0];
            vec[1] = flatxy[indhn][1] - flatxy[indhp][1];
            energy += cotangent[inde] * (pow(new_x(indi) - new_x(indj) - 
                SVDLt[indL](0, 0) * vec[0] -SVDLt[indL](0, 1) * vec[1],2) +
                pow(new_y(indi) - new_y(indj) - SVDLt[indL](1, 0) * vec[0] -
                SVDLt[indL](1, 1) * vec[1],2));
        }
    }
    energy /= 2.0;
    return energy;
}

void USTC_CG::Arap::set_new_mesh()
{
    uv_result.resize(mesh->n_vertices());
    for (int i = 0; i < mesh->n_vertices(); i++)
    {
        uv_result[i][0] = new_x(i);
        uv_result[i][1] = new_y(i);
    }
}

void USTC_CG::Arap::reset_mesh()
{
    uv_mesh = uv_result;
}

pxr::VtArray<pxr::GfVec2f> USTC_CG::Arap::get_new_mesh()
{
    return uv_result;
}
}