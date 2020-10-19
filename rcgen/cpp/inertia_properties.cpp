#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

ur5::rcg::InertiaProperties::InertiaProperties()
{
    com_shoulder = Vector3(0.0,comy_shoulder,comz_shoulder);
    tensor_shoulder.fill(
        m_shoulder,
        com_shoulder,
        Utils::buildInertiaTensor<Scalar>(ix_shoulder,iy_shoulder,iz_shoulder,0.0,0.0,iyz_shoulder) );

    com_upper_arm = Vector3(0.0,comy_upper_arm,comz_upper_arm);
    tensor_upper_arm.fill(
        m_upper_arm,
        com_upper_arm,
        Utils::buildInertiaTensor<Scalar>(ix_upper_arm,iy_upper_arm,iz_upper_arm,0.0,0.0,iyz_upper_arm) );

    com_forearm = Vector3(0.0,comy_forearm,comz_forearm);
    tensor_forearm.fill(
        m_forearm,
        com_forearm,
        Utils::buildInertiaTensor<Scalar>(ix_forearm,iy_forearm,iz_forearm,0.0,0.0,iyz_forearm) );

    com_wrist_1 = Vector3(0.0,comy_wrist_1,comz_wrist_1);
    tensor_wrist_1.fill(
        m_wrist_1,
        com_wrist_1,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_1,iy_wrist_1,iz_wrist_1,0.0,0.0,iyz_wrist_1) );

    com_wrist_2 = Vector3(0.0,comy_wrist_2,comz_wrist_2);
    tensor_wrist_2.fill(
        m_wrist_2,
        com_wrist_2,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_2,iy_wrist_2,iz_wrist_2,0.0,0.0,iyz_wrist_2) );

    com_wrist_3 = Vector3(0.0,comy_wrist_3,0.0);
    tensor_wrist_3.fill(
        m_wrist_3,
        com_wrist_3,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_3,iy_wrist_3,iz_wrist_3,0.0,0.0,0.0) );

}


void ur5::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
