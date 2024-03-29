#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"
#include <Eigen/Sparse>
#include "utils/util_asap.h"
#include <vector>

namespace USTC_CG::node_asap {

static void node_asap_declare(NodeDeclarationBuilder& b)
{
    b.add_input<decl::Geometry>("Input");
    b.add_input<decl::Geometry>("Origin Mesh");
    b.add_input<decl::Int>("fix index1").min(0).max(3000).default_val(0);
    b.add_input<decl::Int>("fix index2").min(0).max(3000).default_val(0);

    b.add_output<decl::Float2Buffer>("OutputUV");
}

static void node_asap_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");
    auto input2 = params.get_input<GOperandBase>("Origin Mesh");
    // Avoid processing the node when there is no input
    if (!(input.get_component<MeshComponent>() && input2.get_component<MeshComponent>())) {
        throw std::runtime_error("Need Geometry Input.");
    }
    int fix1 = params.get_input<int>("fix index1");
    int fix2 = params.get_input<int>("fix index2");

    auto halfedge_mesh = operand_to_openmesh(&input);
    auto origin_mesh = operand_to_openmesh(&input2);

    Asap* asap = new Asap;
    double energy = 0, energy_new = 1;
    asap->init(origin_mesh, halfedge_mesh);
    asap->set_uv_mesh();
    asap->set_flatxy();
    asap->set_fixed(fix1, fix2);
    asap->set_cotangent();
    asap->set_matrixA();

    asap->set_Laplacian();
    energy_new = asap->energy_cal();
    asap->set_new_mesh();
    asap->reset_mesh();

    int flag = 0;
    while (fabs(energy - energy_new) > 0.01 && flag < 3) {
        energy = energy_new;
        asap->set_Laplacian();
        energy_new = asap->energy_cal();
        asap->set_new_mesh();
        asap->reset_mesh();
        flag++;
    }

    // The result UV coordinates
    pxr::VtArray<pxr::GfVec2f> uv_result;
    uv_result = asap->get_new_mesh();
    std::vector<float> x(uv_result.size()), y(uv_result.size());
    for (int i = 0; i < x.size(); i++) {
        x[i] = uv_result[i][0];
        y[i] = uv_result[i][1];
    }


    params.set_output("OutputUV", uv_result);
}

static void node_register()
{
    static NodeTypeInfo ntype;

    strcpy(ntype.ui_name, "ASAP Parameterization");
    strcpy_s(ntype.id_name, "geom_asap");

    geo_node_type_base(&ntype);
    ntype.node_execute = node_asap_exec;
    ntype.declare = node_asap_declare;
    nodeRegisterType(&ntype);
}

NOD_REGISTER_NODE(node_register)
}