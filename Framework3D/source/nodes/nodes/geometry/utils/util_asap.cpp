#include "util_asap.h"
#include <vector>

namespace USTC_CG
{
void USTC_CG::Asap::init(std::shared_ptr<PolyMesh> mesh1, std::shared_ptr<PolyMesh> mesh2)
{
    origin_mesh = mesh1;
    mesh = mesh2;
}

void USTC_CG::Asap::set_cotangent()
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

void USTC_CG::Asap::set_matrixA()
{
    int nv = mesh->n_vertices();
    int nf = mesh->n_faces();
    A.resize(2 * (nv + nf), 2 * (nv + nf));
    typedef Eigen::Triplet<double> T;
    std::vector<T> tlist;
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (vertex_handle.idx() == fix_ind1 || vertex_handle.idx() == fix_ind2) 
        {
            tlist.push_back(T(vertex_handle.idx(), vertex_handle.idx(), 1));
            tlist.push_back(T(vertex_handle.idx() + nv, vertex_handle.idx() + nv, 1));
        }
        else
        {
            double tmp = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) 
            {
                double tmp1 = cotangent[halfedge_handle.idx()];
                double tmp2 = cotangent[halfedge_handle.opp().idx()];
                double tmp_ = tmp1 + tmp2;
                tmp += tmp_;
                if (!halfedge_handle.is_boundary())
                {
                    int indfij = halfedge_handle.face().idx();
                    tlist.push_back(T(
                        vertex_handle.idx(), indfij + 2 * nv, 
                        -tmp1 * (flatxy[halfedge_handle.next().idx()][0] - 
                            flatxy[halfedge_handle.prev().idx()][0])));
                    tlist.push_back(
                        T(vertex_handle.idx(),indfij + 2 * nv + nf,
                          -tmp1 * (flatxy[halfedge_handle.next().idx()][1] -
                                   flatxy[halfedge_handle.prev().idx()][1])));
                    tlist.push_back(
                        T(vertex_handle.idx() + nv, indfij + 2 * nv,
                          -tmp1 * (flatxy[halfedge_handle.next().idx()][1] -
                                   flatxy[halfedge_handle.prev().idx()][1])));
                    tlist.push_back(
                        T(vertex_handle.idx() + nv, indfij + 2 * nv + nf,
                          tmp1 * (flatxy[halfedge_handle.next().idx()][0] -
                                   flatxy[halfedge_handle.prev().idx()][0])));
                }
                if (!halfedge_handle.opp().is_boundary())
                {
                    int indfji = halfedge_handle.opp().face().idx();
                    tlist.push_back(
                        T(vertex_handle.idx(), indfji + 2 * nv,
                          -tmp2 * (flatxy[halfedge_handle.opp().next().idx()][0] -
                                   flatxy[halfedge_handle.opp().prev().idx()][0])));
                    tlist.push_back(
                        T(vertex_handle.idx(), indfji + 2 * nv + nf,
                          -tmp2 * (flatxy[halfedge_handle.opp().next().idx()][1] -
                                   flatxy[halfedge_handle.opp().prev().idx()][1])));
                    tlist.push_back(
                        T(vertex_handle.idx() + nv, indfji + 2 * nv,
                          -tmp2 * (flatxy[halfedge_handle.opp().next().idx()][1] -
                                   flatxy[halfedge_handle.opp().prev().idx()][1])));
                    tlist.push_back(
                        T(vertex_handle.idx() + nv, indfji + 2 * nv + nf,
                          tmp2 * (flatxy[halfedge_handle.opp().next().idx()][0] -
                                  flatxy[halfedge_handle.opp().prev().idx()][0])));
                }
                if (halfedge_handle.to().idx() != fix_ind1 && halfedge_handle.to().idx() != fix_ind2)
                {
                    tlist.push_back(T(vertex_handle.idx(), halfedge_handle.to().idx(), -tmp_));
                    tlist.push_back(
                        T(vertex_handle.idx() + nv, halfedge_handle.to().idx() + nv, -tmp_));
                }
            }
            tlist.push_back(T(vertex_handle.idx(), vertex_handle.idx(), tmp));
            tlist.push_back(T(vertex_handle.idx() + nv, vertex_handle.idx() + nv, tmp));
        }
    }
    for (const auto& face_handle : origin_mesh->faces())
    {
        int indf = face_handle.idx();
        const auto& halfedge_handle = face_handle.halfedge();
        int ind1 = halfedge_handle.from().idx(), ind2 = halfedge_handle.to().idx(),
            ind3 = halfedge_handle.next().to().idx();
        int indh = halfedge_handle.idx(), indp = halfedge_handle.prev().idx(),
            indn = halfedge_handle.next().idx();
        double tmp1 = cotangent[indh] * (pow(flatxy[indn][0] - flatxy[indp][0], 2) +
                                         pow(flatxy[indn][1] - flatxy[indp][1], 2)) +
                      cotangent[indp] * (pow(flatxy[indh][0] - flatxy[indn][0], 2) +
                                         pow(flatxy[indh][1] - flatxy[indn][1], 2)) +
                      cotangent[indn] * (pow(flatxy[indp][0] - flatxy[indh][0], 2) +
                                         pow(flatxy[indp][1] - flatxy[indh][1], 2));
        tlist.push_back(T(2 * nv + indf, 2 * nv + indf, tmp1));
        tlist.push_back(T(2 * nv + indf + nf, 2 * nv + indf + nf, tmp1));
        if (ind1 != fix_ind1 && ind1 != fix_ind2)
        {
            double t1 = cotangent[indp] * (flatxy[indh][0] - flatxy[indn][0]) -
                        cotangent[indh] * (flatxy[indn][0] - flatxy[indp][0]);
            double t2 = cotangent[indp] * (flatxy[indh][1] - flatxy[indn][1]) -
                        cotangent[indh] * (flatxy[indn][1] - flatxy[indp][1]);
            tlist.push_back(T(2 * nv + indf, ind1, t1));
            tlist.push_back(T(2 * nv + indf, ind1 + nv, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind1, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind1 + nv, -t1));
        }
        if (ind2 != fix_ind1 && ind2 != fix_ind2) 
        {
            double t1 = cotangent[indh] * (flatxy[indn][0] - flatxy[indp][0]) -
                        cotangent[indn] * (flatxy[indp][0] - flatxy[indh][0]);
            double t2 = cotangent[indh] * (flatxy[indn][1] - flatxy[indp][1]) -
                        cotangent[indn] * (flatxy[indp][1] - flatxy[indh][1]);
            tlist.push_back(T(2 * nv + indf, ind2, t1));
            tlist.push_back(T(2 * nv + indf, ind2 + nv, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind2, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind2 + nv, -t1));
        }
        if (ind3 != fix_ind1 && ind3 != fix_ind2) 
        {
            double t1 = cotangent[indn] * (flatxy[indp][0] - flatxy[indh][0]) -
                        cotangent[indp] * (flatxy[indh][0] - flatxy[indn][0]);
            double t2 = cotangent[indn] * (flatxy[indp][1] - flatxy[indh][1]) -
                        cotangent[indp] * (flatxy[indh][1] - flatxy[indn][1]);
            tlist.push_back(T(2 * nv + indf, ind3, t1));
            tlist.push_back(T(2 * nv + indf, ind3 + nv, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind3, t2));
            tlist.push_back(T(2 * nv + indf + nf, ind3 + nv, -t1));
        }
    }
    A.setFromTriplets(tlist.begin(), tlist.end());
    solver.compute(A);
}

void USTC_CG::Asap::set_fixed(int a, int b)
{
    fix_ind1 = a;
    fix_ind2 = b;
}

void USTC_CG::Asap::set_uv_mesh()
{
    uv_mesh.resize(mesh->n_vertices());
    for (const auto& vertex_handle : mesh->vertices()) {
        const auto& vec = mesh->point(vertex_handle);
        uv_mesh[vertex_handle.idx()][0] = vec[1];
        uv_mesh[vertex_handle.idx()][1] = vec[2];
    }
}

void USTC_CG::Asap::set_flatxy()
{
    flatxy.resize(mesh->n_halfedges());
    // caution reverse triangle!!!
    for (const auto& face_handle : origin_mesh->faces()) {
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
                (uv_mesh[ind0][0] - uv_mesh[ind2][0]) * (uv_mesh[ind0][1] - uv_mesh[ind1][1]) >=
            0)
            flatxy[halfedge_handle.prev().idx()][1] = (xy[0] - xy[2]).norm() * sin(acos(cos_));
        else
            flatxy[halfedge_handle.prev().idx()][1] = -(xy[0] - xy[2]).norm() * sin(acos(cos_));
    }
}

void USTC_CG::Asap::set_Laplacian()
{
    int nv = mesh->n_vertices();
    int nf = mesh->n_faces();

    xyab.resize(2 * nv + 2 * nf);
    result.resize(2 * nv + 2 * nf);
    for (const auto& vertex_handle : mesh->vertices())
    {
        int indx = vertex_handle.idx();
        if (indx == fix_ind1 || indx == fix_ind2)
        {
            xyab(indx) = uv_mesh[indx][0];
            xyab(indx + nv) = uv_mesh[indx][1]; 
        }
        else
        {
            xyab(indx) = 0;
            xyab(indx + nv) = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
            {
                int indh = halfedge_handle.to().idx();
                int indij = halfedge_handle.idx();
                int indji = halfedge_handle.opp().idx();
                if (indh == fix_ind1 || indh == fix_ind2)
                {
                    xyab(indx) += (cotangent[indij] + cotangent[indji]) * uv_mesh[indh][0];
                    xyab(indx + nv) += (cotangent[indij] + cotangent[indji]) * uv_mesh[indh][1];
                }
            }
        }
    }
    for (const auto& face_handle : mesh->faces())
    {
        int indf = face_handle.idx();
        xyab(2 * nv + indf) = 0;
        xyab(2 * nv + indf + nf) = 0;
        const auto& halfedge_handle = face_handle.halfedge();
        int ind1 = halfedge_handle.from().idx(), ind2 = halfedge_handle.to().idx(),
            ind3 = halfedge_handle.next().to().idx();
        int indh = halfedge_handle.idx(), indp = halfedge_handle.prev().idx(),
            indn = halfedge_handle.next().idx();
        if (ind1 == fix_ind1 || ind1 == fix_ind2)
        {
            double t1 = cotangent[indp] * (flatxy[indh][0] - flatxy[indn][0]) -
                        cotangent[indh] * (flatxy[indn][0] - flatxy[indp][0]);
            double t2 = cotangent[indp] * (flatxy[indh][1] - flatxy[indn][1]) -
                        cotangent[indh] * (flatxy[indn][1] - flatxy[indp][1]);
            xyab(2 * nv + indf) += -t1 * uv_mesh[ind1][0] - t2 * uv_mesh[ind1][1];
            xyab(2 * nv + indf + nf) += -t2 * uv_mesh[ind1][0] + t1 * uv_mesh[ind1][1];
        }
        if (ind2 == fix_ind1 || ind2 == fix_ind2) {
            double t1 = cotangent[indh] * (flatxy[indn][0] - flatxy[indp][0]) -
                         cotangent[indn] * (flatxy[indp][0] - flatxy[indh][0]);
            double t2 = cotangent[indh] * (flatxy[indn][1] - flatxy[indp][1]) -
                        cotangent[indn] * (flatxy[indp][1] - flatxy[indh][1]);
            xyab(2 * nv + indf) += -t1 * uv_mesh[ind2][0] - t2 * uv_mesh[ind2][1];
            xyab(2 * nv + indf + nf) += -t2 * uv_mesh[ind2][0] + t1 * uv_mesh[ind2][1];
        }
        if (ind3 == fix_ind1 || ind3 == fix_ind2) {
            double t1 = cotangent[indn] * (flatxy[indh][0] - flatxy[indp][0]) -
                        cotangent[indp] * (flatxy[indh][0] - flatxy[indn][0]);
            double t2 = cotangent[indn] * (flatxy[indh][1] - flatxy[indp][1]) -
                        cotangent[indp] * (flatxy[indh][1] - flatxy[indn][1]);
            xyab(2 * nv + indf) += -t1 * uv_mesh[ind3][0] - t2 * uv_mesh[ind3][1];
            xyab(2 * nv + indf + nf) += -t2 * uv_mesh[ind3][0] + t1 * uv_mesh[ind3][1];
        }
    }
    result = solver.solve(xyab);
}

double USTC_CG::Asap::energy_cal()
{
    energy = 0;
    int nv = mesh->n_vertices();
    int nf = mesh->n_faces();
    for (const auto& halfedge_handle : mesh->halfedges())
    {
        if (!halfedge_handle.is_boundary())
        {
            int indh = halfedge_handle.idx();
            int indi = halfedge_handle.from().idx();
            int indj = halfedge_handle.to().idx();
            int indf = halfedge_handle.face().idx();
            int indp = halfedge_handle.prev().idx();
            int indn = halfedge_handle.next().idx();
            double a = result(2 * nv + indf);
            double b = result(2 * nv + nf + indf);
            energy += cotangent[indh] *
                (pow(result(indi) - result(indj) - a * (flatxy[indn][0] - flatxy[indp][0]) -
                 b * (flatxy[indn][1] - flatxy[indp][1]), 2) + pow(result(indi + nv) - result(indj + nv) +
                 b * (flatxy[indn][0] - flatxy[indp][0]) - a * (flatxy[indn][1] - flatxy[indp][1]), 2));
        }
    }
    energy /= 2;
    return energy;
}

void USTC_CG::Asap::set_new_mesh()
{
    int nv = mesh->n_vertices();
    uv_result.resize(nv);
    for (int i = 0; i < nv; i++) {
        uv_result[i][0] = result(i);
        uv_result[i][1] = result(i + nv);
    }
}

void USTC_CG::Asap::reset_mesh()
{
    uv_mesh = uv_result;
}

pxr::VtArray<pxr::GfVec2f> USTC_CG::Asap::get_new_mesh()
{
    return uv_result;
}
}