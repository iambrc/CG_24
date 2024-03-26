#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <vector>

/*
** @brief HW4_TutteParameterization
**
** This file presents the basic framework of a "node", which processes inputs
** received from the left and outputs specific variables for downstream nodes to
** use.
** - In the first function, node_declare, you can set up the node's input and
** output variables.
** - The second function, node_exec is the execution part of the node, where we
** need to implement the node's functionality.
** - The third function generates the node's registration information, which
** eventually allows placing this node in the GUI interface.
**
** Your task is to fill in the required logic at the specified locations
** within this template, espacially in node_exec.
*/

namespace USTC_CG::node_min_surf {
static void node_min_surf_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");

    // Output-1: Minimal surface with fixed boundary
    b.add_output<decl::Geometry>("Output");
}

static void node_min_surf_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    
    OpenMesh::VPropHandleT<int> ind;
    halfedge_mesh->add_property(ind, "NotBoundaryIndex");
    int not_boundary_count = 0;
    size_t len = halfedge_mesh->n_vertices(), size_m; // size_m is the size of matrix
    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        if (vertex_handle.is_boundary())
            halfedge_mesh->property(ind, vertex_handle) = -1;
        else
            halfedge_mesh->property(ind, vertex_handle) = not_boundary_count++;
    }
    size_m = not_boundary_count;
    Eigen::SparseMatrix<double> A(size_m, size_m);
    Eigen::VectorXd bx(size_m), by(size_m), bz(size_m), new_x(size_m), new_y(size_m), new_z(size_m);
    typedef Eigen::Triplet<double> T;
    std::vector<T> tlist;

    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        const auto& position = halfedge_mesh->point(vertex_handle);
        if (!vertex_handle.is_boundary())
        {
            int current_ind = halfedge_mesh->property(ind, vertex_handle);
            bx(current_ind) = 0.0;
            by(current_ind) = 0.0;
            bz(current_ind) = 0.0;
            tlist.push_back(T(current_ind,current_ind,1.0));
            int edge_num = int(vertex_handle.edges().to_vector().size());
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
            {
                const auto& v = halfedge_handle.to();
                const auto& vec = halfedge_mesh->point(v);
                if (halfedge_handle.to().is_boundary())
                {
                    // w_i = 1/N
                    bx(current_ind) += vec[0] / edge_num;
                    by(current_ind) += vec[1] / edge_num;
                    bz(current_ind) += vec[2] / edge_num;
                }
                else
                {
                    tlist.push_back(T(current_ind,
                    halfedge_mesh->property(ind, halfedge_handle.to()),-1.0/edge_num));
                }
            }
        }
    }
    A.setFromTriplets(tlist.begin(), tlist.end());
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    new_x = solver.solve(bx);
    new_y = solver.solve(by);
    new_z = solver.solve(bz);

    //set new vertices
    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        if (!vertex_handle.is_boundary())
        {
            OpenMesh::DefaultTraits::Point new_point;
            new_point[0] = new_x(halfedge_mesh->property(ind, vertex_handle));
            new_point[1] = new_y(halfedge_mesh->property(ind, vertex_handle));
            new_point[2] = new_z(halfedge_mesh->property(ind, vertex_handle));
            halfedge_mesh->set_point(vertex_handle, new_point);
        }
    }
    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}

static void node_min_surf_cotan_declare(NodeDeclarationBuilder& b)
{
    // Input-1: boundary-changed mesh
    b.add_input<decl::Geometry>("Input");

    // Input-2: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Origin Mesh");

    // Output-1: Minimal surface with fixed boundary
    b.add_output<decl::Geometry>("Output");
}

static void node_min_surf_cotan_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto input2 = params.get_input<GOperandBase>("Origin Mesh");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!(input.get_component<MeshComponent>() && input2.get_component<MeshComponent>())) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
    }
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto origin_mesh = operand_to_openmesh(&input2);


    OpenMesh::VPropHandleT<int> ind;
    origin_mesh->add_property(ind, "NotBoundaryIndex");
    int not_boundary_count = 0;
    size_t len = origin_mesh->n_vertices(), size_m; // size_m is the size of matrix
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (vertex_handle.is_boundary())
            origin_mesh->property(ind, vertex_handle) = -1;
        else
            origin_mesh->property(ind, vertex_handle) = not_boundary_count++;
    }
    size_m = not_boundary_count;
    Eigen::SparseMatrix<double> A(size_m, size_m);
    Eigen::VectorXd bx(size_m), by(size_m), bz(size_m), new_x(size_m), new_y(size_m), new_z(size_m);
    typedef Eigen::Triplet<double> T;
    std::vector<T> tlist;

    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (!vertex_handle.is_boundary())
        {
            int current_ind = origin_mesh->property(ind, vertex_handle);
            bx(current_ind) = 0.0;
            by(current_ind) = 0.0;
            bz(current_ind) = 0.0;
            tlist.push_back(T(current_ind,current_ind,1.0));
            int edge_num = int(vertex_handle.edges().to_vector().size());
            double total_w = 0.0;
            pxr::VtArray<double> w(edge_num);
            size_t itr = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
            {
                // calculate weight
                const auto& v1 = halfedge_handle.next().to();
                const auto& v2 = halfedge_handle.to();
                const auto& v3 = halfedge_handle.opp().prev().from();
                const auto& vec1 = origin_mesh->point(v1) - origin_mesh->point(vertex_handle);
                const auto& vec2 = origin_mesh->point(v1) - origin_mesh->point(v2);
                const auto& vec3 = origin_mesh->point(v3) - origin_mesh->point(vertex_handle);
                const auto& vec4 = origin_mesh->point(v3) - origin_mesh->point(v2);
                double cos1 = vec1.dot(vec2) / (vec1.norm()*vec2.norm());
                double cos2 = vec3.dot(vec4) / (vec3.norm()*vec4.norm());
                w[itr] = 1.0/tan(acos(cos1)) + 1.0/tan(acos(cos2));
                total_w += w[itr];
                itr++;
            }
            itr = 0;
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges())
            {
                const auto& v = halfedge_handle.to();
                OpenMesh::VertexHandle vh = halfedge_mesh->vertex_handle(v.idx());
                const auto& vec = halfedge_mesh->point(vh);
                if (halfedge_handle.to().is_boundary())
                {
                    // w_i = cot alpha + cot beta
                    bx(current_ind) += w[itr] * vec[0] / total_w;
                    by(current_ind) += w[itr] * vec[1] / total_w;
                    bz(current_ind) += w[itr] * vec[2] / total_w;
                }
                else
                {
                    tlist.push_back(T(current_ind,
                    origin_mesh->property(ind, halfedge_handle.to()),-w[itr] / total_w));
                }
                itr++;
            }
        }
    }
    A.setFromTriplets(tlist.begin(), tlist.end());
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    new_x = solver.solve(bx);
    new_y = solver.solve(by);
    new_z = solver.solve(bz);

    //set new vertices
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (!vertex_handle.is_boundary())
        {
            OpenMesh::DefaultTraits::Point new_point;
            new_point[0] = new_x(origin_mesh->property(ind, vertex_handle));
            new_point[1] = new_y(origin_mesh->property(ind, vertex_handle));
            new_point[2] = new_z(origin_mesh->property(ind, vertex_handle));
            OpenMesh::VertexHandle vh = halfedge_mesh->vertex_handle(vertex_handle.idx());
            halfedge_mesh->set_point(vh, new_point);
        }
    }

    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}

static void node_min_surf_shape_preserving_declare(NodeDeclarationBuilder& b)
{
    // Input-1: boundary-changed mesh
    b.add_input<decl::Geometry>("Input");

    // Input-2: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Origin Mesh");

    // Output-1: Minimal surface with fixed boundary
    b.add_output<decl::Geometry>("Output");
}

static void node_min_surf_shape_preserving_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto input2 = params.get_input<GOperandBase>("Origin Mesh");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!(input.get_component<MeshComponent>() && input2.get_component<MeshComponent>())) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
    }
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto origin_mesh = operand_to_openmesh(&input2);

    
    OpenMesh::VPropHandleT<int> ind;
    origin_mesh->add_property(ind, "NotBoundaryIndex");
    int not_boundary_count = 0;
    size_t len = origin_mesh->n_vertices(), size_m; // size_m is the size of matrix
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (vertex_handle.is_boundary())
            origin_mesh->property(ind, vertex_handle) = -1;
        else
            origin_mesh->property(ind, vertex_handle) = not_boundary_count++;
    }
    size_m = not_boundary_count;
    Eigen::SparseMatrix<double> A(size_m, size_m);
    Eigen::VectorXd bx(size_m), by(size_m), bz(size_m), new_x(size_m), new_y(size_m), new_z(size_m);
    typedef Eigen::Triplet<double> T;
    std::vector<T> tlist;

    // next we need to calculate weight at each point
    std::vector<double> w;
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (!vertex_handle.is_boundary())
        {
            int current_ind = origin_mesh->property(ind, vertex_handle);
            bx(current_ind) = 0.0;
            by(current_ind) = 0.0;
            bz(current_ind) = 0.0;
            tlist.push_back(T(current_ind,current_ind,1.0));
            int edge_num = 0;
            double total_w = 0.0;

            // calculate weight
            pxr::VtArray<OpenMesh::SmartVertexHandle> vi{};
            for (const auto& itr_ : vertex_handle.vertices_ccw())
            {
                vi.push_back(itr_);
                edge_num++;
            }
            for (const auto& itr_ : vertex_handle.vertices_ccw())
            {
                // here we need to ensure vi[n+1] = v[1]
                vi.push_back(itr_);
                break;
            }
            w.resize(edge_num);
            for (size_t i = 0; i < edge_num; i++)
                w[i] = 0;
            // first we calculate the angle between x_jk and x_i
            double total_angle = 0;
            pxr::VtArray<double> angle(edge_num);
            for (int k = 0; k < edge_num; k++)
            {
                const auto& vec1 = origin_mesh->point(vertex_handle) - origin_mesh->point(vi[k]);
                const auto& vec2 = origin_mesh->point(vertex_handle) - origin_mesh->point(vi[k+1]);
                if (vec1.norm()==0 || vec2.norm()==0)
                    angle[k] = 0;
                else
                    angle[k] = acos(vec1.dot(vec2)/(vec1.norm()*vec2.norm()));
                total_angle += angle[k];
            }
            // then we calculate p_i, p = 0, p_1 = (||x_j1-x_i||,0)
            pxr::VtArray<double> px(edge_num+1), py(edge_num+1), argp(edge_num);
            px[0] = (origin_mesh->point(vi[0])-origin_mesh->point(vertex_handle)).norm(), py[0]=0;
            argp[0] = 0;
            for (int k = 1; k < edge_num; k++)
            {
                argp[k] = argp[k-1] + 2*acos(-1.0)*(angle[k])/total_angle;
                double len_p = (origin_mesh->point(vi[k])-origin_mesh->point(vertex_handle)).norm();
                px[k] = len_p*cos(argp[k]);
                py[k] = len_p*sin(argp[k]);
            }
            px[edge_num] = px[0], py[edge_num] = py[0];
            if (edge_num == 3)
            {
                // only three neibors, calculate weight by area directly
                w[0] = 0.5*fabs(px[2]*py[0]-px[0]*py[2]);
                w[1] = 0.5*fabs(px[1]*py[0]-px[0]*py[1]);
                w[2] = 0.5*fabs(px[1]*py[2]-px[2]*py[1]);
                total_w = w[0] + w[1] + w[2];
            }
            else
            {
                // edge_num > 3, use other method in the article Floater97
                // we find a triangle s.t. p in pr pr+1 for each pl
                double delta1, delta2, delta3;
                for (int k = 0; k < edge_num; k++)
                {
                    for (int it_ = 0; it_ < edge_num; it_++)
                    {
                        delta1 = delta2 = delta3 = 0;
                        if ((px[k]-px[it_+1])*(py[it_]-py[it_+1])-(px[it_]-px[it_+1])*(py[k]-py[it_+1]) != 0)
                        {
                            // caution the accuracy
                            if (fabs(px[it_]-px[it_+1]) > fabs(py[it_]-py[it_+1]))
                            {
                                double tmp = (py[it_]-py[it_+1]) / (px[it_]-px[it_+1]);
                                delta1 = (-px[it_+1]*tmp+py[it_+1])/((px[k]-px[it_+1])*tmp-(py[k]-py[it_+1]));
                            }
                            else
                            {
                                double tmp = (px[it_]-px[it_+1]) / (py[it_]-py[it_+1]);
                                delta1 = (-px[it_+1]+py[it_+1]*tmp)/((px[k]-px[it_+1])-(py[k]-py[it_+1])*tmp);
                            }
                            if (fabs(px[k]-px[it_+1]) > fabs(py[k]-py[it_+1]))
                            {
                                double tmp = (py[k]-py[it_+1]) / (px[k]-px[it_+1]);
                                delta2 = (-py[it_+1]+px[it_+1]*tmp)/((py[it_]-py[it_+1])-(px[it_]-px[it_+1])*tmp);
                            }
                            else
                            {
                                double tmp = (px[k]-px[it_+1]) / (py[k]-py[it_+1]);
                                delta2 = (-py[it_+1]*tmp+px[it_+1])/((py[it_]-py[it_+1])*tmp-(px[it_]-px[it_+1]));
                            }
                            delta3 = 1.0 - delta1 - delta2;
                        }
                        else if (fabs(px[k]*py[it_]-px[it_]*py[k]) < 1e-8 && it_ != k)
                        {
                            delta3 = 0;
                            delta1 = (sqrt(px[it_]*px[it_]+py[it_]*py[it_]))/
                                (sqrt(px[it_]*px[it_]+py[it_]*py[it_])+sqrt(px[k]*px[k]+py[k]*py[k]));
                            delta2 = 1.0 - delta1;
                        }

                        if (delta3 >= 0 && delta1 > 0 && delta2 > 0)
                        {
                            w[k] += delta1;
                            w[it_] += delta2;
                            w[(it_+1)%edge_num] += delta3;
                            break;
                        }
                        else
                        {
                            if(it_ == edge_num - 1)
                            {
                                // still cannot find for some vertex
                                for (int j = 0; j < edge_num; j++)
                                    w[j] += 1.0/edge_num;
                            }
                        }
                    }
                }
                total_w = double(edge_num);
            }
            size_t itr = 0;
            for (const auto& vt : vertex_handle.vertices_ccw())
            {
                OpenMesh::VertexHandle vh = halfedge_mesh->vertex_handle(vt.idx());
                const auto& vec = halfedge_mesh->point(vh);
                if (vt.is_boundary())
                {
                    // shape preserving
                    bx(current_ind) += w[itr] * vec[0] / total_w;
                    by(current_ind) += w[itr] * vec[1] / total_w;
                    bz(current_ind) += w[itr] * vec[2] / total_w;
                }
                else
                {
                    tlist.push_back(T(current_ind,
                    origin_mesh->property(ind, vt),-w[itr] / total_w));
                }
                itr++;
            }
            
        }

    }
    A.setFromTriplets(tlist.begin(), tlist.end());
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    new_x = solver.solve(bx);
    new_y = solver.solve(by);
    new_z = solver.solve(bz);

    //set new vertices
    for (const auto& vertex_handle : origin_mesh->vertices())
    {
        if (!vertex_handle.is_boundary())
        {
            OpenMesh::DefaultTraits::Point new_point;
            new_point[0] = new_x(origin_mesh->property(ind, vertex_handle));
            new_point[1] = new_y(origin_mesh->property(ind, vertex_handle));
            new_point[2] = new_z(origin_mesh->property(ind, vertex_handle));
            OpenMesh::VertexHandle vh = halfedge_mesh->vertex_handle(vertex_handle.idx());
            halfedge_mesh->set_point(vh, new_point);
        }
    }

    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}



static void node_register()
{
    static NodeTypeInfo ntype;

    strcpy(ntype.ui_name, "Minimal Surface");
    strcpy_s(ntype.id_name, "geom_min_surf");

    geo_node_type_base(&ntype);
    ntype.node_execute = node_min_surf_exec;
    ntype.declare = node_min_surf_declare;
    nodeRegisterType(&ntype);

    static NodeTypeInfo ntype2;

    strcpy(ntype2.ui_name, "Minimal Surface Cotangent");
    strcpy_s(ntype2.id_name, "geom_min_surf_cotangent");

    geo_node_type_base(&ntype2);
    ntype2.node_execute = node_min_surf_cotan_exec;
    ntype2.declare = node_min_surf_cotan_declare;
    nodeRegisterType(&ntype2);

    static NodeTypeInfo ntype3;

    strcpy(ntype3.ui_name, "Minimal Surface Shape Preserving");
    strcpy_s(ntype3.id_name, "geom_min_surf_shape_preserving");

    geo_node_type_base(&ntype3);
    ntype3.node_execute = node_min_surf_shape_preserving_exec;
    ntype3.declare = node_min_surf_shape_preserving_declare;
    nodeRegisterType(&ntype3);
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_min_surf
