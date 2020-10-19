#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace ur5::rcg;

Vector3 ur5::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base_X_fr_shoulder;
    tmpSum += inertiaProps.getMass_shoulder() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_shoulder()));
    
    tmpX = tmpX * ht.fr_shoulder_X_fr_upper_arm;
    tmpSum += inertiaProps.getMass_upper_arm() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_upper_arm()));
    
    tmpX = tmpX * ht.fr_upper_arm_X_fr_forearm;
    tmpSum += inertiaProps.getMass_forearm() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_forearm()));
    
    tmpX = tmpX * ht.fr_forearm_X_fr_wrist_1;
    tmpSum += inertiaProps.getMass_wrist_1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_1()));
    
    tmpX = tmpX * ht.fr_wrist_1_X_fr_wrist_2;
    tmpSum += inertiaProps.getMass_wrist_2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_2()));
    
    tmpX = tmpX * ht.fr_wrist_2_X_fr_wrist_3;
    tmpSum += inertiaProps.getMass_wrist_3() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_3()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 ur5::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_shoulder(q);
    ht.fr_shoulder_X_fr_upper_arm(q);
    ht.fr_upper_arm_X_fr_forearm(q);
    ht.fr_forearm_X_fr_wrist_1(q);
    ht.fr_wrist_1_X_fr_wrist_2(q);
    ht.fr_wrist_2_X_fr_wrist_3(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
