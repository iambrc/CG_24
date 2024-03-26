#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include <cmath>
#include <Eigen/Sparse>

/*
** @brief HW4_TutteParameterization
**
** This file contains two nodes whose primary function is to map the boundary of a mesh to a plain
** convex closed curve (circle of square), setting the stage for subsequent Laplacian equation
** solution and mesh parameterization tasks.
**
** Key to this node's implementation is the adept manipulation of half-edge data structures
** to identify and modify the boundary of the mesh.
**
** Task Overview:
** - The two execution functions (node_map_boundary_to_square_exec,
** node_map_boundary_to_circle_exec) require an update to accurately map the mesh boundary to a and
** circles. This entails identifying the boundary edges, evenly distributing boundary vertices along
** the square's perimeter, and ensuring the internal vertices' positions remain unchanged.
** - A focus on half-edge data structures to efficiently traverse and modify mesh boundaries.
*/

namespace USTC_CG::node_boundary_mapping {

static void node_map_boundary_to_circle_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_circle_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }
    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    
    double total_len = 0.0;
    pxr::VtArray<double> edge_len{};
    OpenMesh::SmartVertexHandle current_vertex;
    // first we find the first edge vertex
    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        if (vertex_handle.is_boundary())
        {
            current_vertex = vertex_handle;
            break;
        }
    }
    // then we can loop all the egde vertex by order
    OpenMesh::SmartVertexHandle edge_vertex = current_vertex;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                const auto& v1 = halfedge_mesh->point(edge_vertex);
                const auto& v2 = halfedge_mesh->point(halfedge_handle.to());
                float l = (v1-v2).norm();
                edge_len.push_back(l);
                total_len += l;
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != current_vertex);

    // set the boundary vertex to the circle
    double current_len = 0.0, Radius = 1.0;
    size_t i = 0;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                OpenMesh::DefaultTraits::Point new_point;
                new_point[0] = Radius*cos((current_len/total_len)*2*acos(-1.0));
                new_point[1] = Radius*sin((current_len/total_len)*2*acos(-1.0));
                new_point[2] = 0;
                current_len += edge_len[i++];
                halfedge_mesh->set_point(edge_vertex, new_point);
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != current_vertex);
    
    /* ----------------------------- Postprocess ------------------------------
    ** Convert the result mesh from the halfedge structure back to GOperandBase format as the node's
    ** output.
    */
    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    auto& output = input;
    output.get_component<MeshComponent>()->vertices =
        operand_base->get_component<MeshComponent>()->vertices;

    // Set the output of the nodes
    params.set_output("Output", std::move(output));
}

static void node_map_boundary_to_square_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_square_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Input does not contain a mesh");
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    double total_len = 0.0;
    pxr::VtArray<double> edge_len{};
    // we need find four vertices to fix the corner of the square
    OpenMesh::SmartVertexHandle vertex_1, vertex_2, vertex_3, vertex_4;
    // first we find the first vertex
    for (const auto& vertex_handle : halfedge_mesh->vertices())
    {
        if (vertex_handle.is_boundary())
        {
            vertex_1 = vertex_handle;
            break;
        }
    }
    // then we calculate the length of each edge
    OpenMesh::SmartVertexHandle edge_vertex = vertex_1;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                const auto& v1 = halfedge_mesh->point(edge_vertex);
                const auto& v2 = halfedge_mesh->point(halfedge_handle.to());
                float l = (v1-v2).norm();
                edge_len.push_back(l);
                total_len += l;
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_1);

    // next step: find other three corners of the square through the length of edges
    size_t i = 0;
    double current_len = 0.0;
    double total_len_1, total_len_2, total_len_3, total_len_4;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                if (current_len <= total_len/4 && current_len+edge_len[i] >= total_len/4)
                {
                    total_len_1 = total_len/4 - current_len < current_len+edge_len[i] - total_len/4 ?
                         current_len : current_len+edge_len[i];
                    // compare which vertex is closer to the corner
                    if (total_len_1 == current_len)
                        vertex_2 = edge_vertex;
                    else
                        vertex_2 = halfedge_handle.to();
                }
                if (current_len <= total_len/2 && current_len+edge_len[i] >= total_len/2)
                {
                    total_len_2 = total_len/2 - current_len < current_len+edge_len[i] - total_len/2 ?
                        current_len : current_len+edge_len[i];
                    // compare which vertex is closer to the corner
                    if (total_len_2 == current_len)
                        vertex_3 = edge_vertex;
                    else
                        vertex_3 = halfedge_handle.to();
                    total_len_2 -= total_len_1;
                }
                if (current_len <= 3*total_len/4 && current_len+edge_len[i] >= 3*total_len/4)
                {
                    total_len_3 = 3*total_len/4 - current_len < current_len+edge_len[i] - 3*total_len/4 ?
                        current_len : current_len+edge_len[i];
                    // compare which vertex is closer to the corner
                    if (total_len_3 == current_len)
                        vertex_4 = edge_vertex;
                    else
                        vertex_4 = halfedge_handle.to();
                    total_len_3 -= total_len_1 + total_len_2;
                }
                current_len += edge_len[i];
                i++;
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_1);
    total_len_4 = total_len - total_len_1 - total_len_2 - total_len_3;

    // next step : cut the boundary into four pieces and map them to the four edges of the square
    double a = 1.0;
    current_len = 0.0;
    i = 0;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                OpenMesh::DefaultTraits::Point new_point;
                new_point[0] = 0;
                new_point[1] = (current_len/total_len_1)*a;
                new_point[2] = 0;
                current_len += edge_len[i++];
                halfedge_mesh->set_point(edge_vertex, new_point);
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_2);
    current_len = 0.0;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                OpenMesh::DefaultTraits::Point new_point;
                new_point[0] = 0;
                new_point[1] = 1.0;
                new_point[2] = (current_len/total_len_2)*a;
                current_len += edge_len[i++];
                halfedge_mesh->set_point(edge_vertex, new_point);
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_3);
    current_len = 0.0;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                OpenMesh::DefaultTraits::Point new_point;
                new_point[0] = 0;
                new_point[1] = a - (current_len/total_len_3)*a;
                new_point[2] = 1.0;
                current_len += edge_len[i++];
                halfedge_mesh->set_point(edge_vertex, new_point);
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_4);
    current_len = 0.0;
    do{
        for (const auto& halfedge_handle : edge_vertex.outgoing_halfedges())
        {
            if (halfedge_handle.is_boundary())
            {
                OpenMesh::DefaultTraits::Point new_point;
                new_point[0] = 0;
                new_point[1] = 0;
                new_point[2] = a - (current_len/total_len_4)*a;
                current_len += edge_len[i++];
                halfedge_mesh->set_point(edge_vertex, new_point);
                edge_vertex = halfedge_handle.to();
                break;
            }
        }
    }while (edge_vertex != vertex_1);

    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}

static void node_register()
{
    static NodeTypeInfo ntype_square, ntype_circle;

    strcpy(ntype_square.ui_name, "Map Boundary to Square");
    strcpy_s(ntype_square.id_name, "geom_map_boundary_to_square");

    geo_node_type_base(&ntype_square);
    ntype_square.node_execute = node_map_boundary_to_square_exec;
    ntype_square.declare = node_map_boundary_to_square_declare;
    nodeRegisterType(&ntype_square);

    strcpy(ntype_circle.ui_name, "Map Boundary to Circle");
    strcpy_s(ntype_circle.id_name, "geom_map_boundary_to_circle");

    geo_node_type_base(&ntype_circle);
    ntype_circle.node_execute = node_map_boundary_to_circle_exec;
    ntype_circle.declare = node_map_boundary_to_circle_declare;
    nodeRegisterType(&ntype_circle);
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_boundary_mapping
