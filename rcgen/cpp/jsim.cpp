#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
ur5::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    wrist_3_Ic(linkInertias.getTensor_wrist_3())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const ur5::rcg::JSIM& ur5::rcg::JSIM::update(const JointState& state) {
    Force F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_wrist_2_X_fr_wrist_3(state);
    frcTransf -> fr_wrist_1_X_fr_wrist_2(state);
    frcTransf -> fr_forearm_X_fr_wrist_1(state);
    frcTransf -> fr_upper_arm_X_fr_forearm(state);
    frcTransf -> fr_shoulder_X_fr_upper_arm(state);

    // Initializes the composite inertia tensors
    shoulder_Ic = linkInertias.getTensor_shoulder();
    upper_arm_Ic = linkInertias.getTensor_upper_arm();
    forearm_Ic = linkInertias.getTensor_forearm();
    wrist_1_Ic = linkInertias.getTensor_wrist_1();
    wrist_2_Ic = linkInertias.getTensor_wrist_2();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link wrist_3:
    iit::rbd::transformInertia<Scalar>(wrist_3_Ic, frcTransf -> fr_wrist_2_X_fr_wrist_3, Ic_spare);
    wrist_2_Ic += Ic_spare;

    F = wrist_3_Ic.col(AZ);
    DATA(WR3, WR3) = F(AZ);

    F = frcTransf -> fr_wrist_2_X_fr_wrist_3 * F;
    DATA(WR3, WR2) = F(AZ);
    DATA(WR2, WR3) = DATA(WR3, WR2);
    F = frcTransf -> fr_wrist_1_X_fr_wrist_2 * F;
    DATA(WR3, WR1) = F(AZ);
    DATA(WR1, WR3) = DATA(WR3, WR1);
    F = frcTransf -> fr_forearm_X_fr_wrist_1 * F;
    DATA(WR3, ELBOW) = F(AZ);
    DATA(ELBOW, WR3) = DATA(WR3, ELBOW);
    F = frcTransf -> fr_upper_arm_X_fr_forearm * F;
    DATA(WR3, SHOULDER_LIFT) = F(AZ);
    DATA(SHOULDER_LIFT, WR3) = DATA(WR3, SHOULDER_LIFT);
    F = frcTransf -> fr_shoulder_X_fr_upper_arm * F;
    DATA(WR3, SHOULDER_PAN) = F(AZ);
    DATA(SHOULDER_PAN, WR3) = DATA(WR3, SHOULDER_PAN);

    // Link wrist_2:
    iit::rbd::transformInertia<Scalar>(wrist_2_Ic, frcTransf -> fr_wrist_1_X_fr_wrist_2, Ic_spare);
    wrist_1_Ic += Ic_spare;

    F = wrist_2_Ic.col(AZ);
    DATA(WR2, WR2) = F(AZ);

    F = frcTransf -> fr_wrist_1_X_fr_wrist_2 * F;
    DATA(WR2, WR1) = F(AZ);
    DATA(WR1, WR2) = DATA(WR2, WR1);
    F = frcTransf -> fr_forearm_X_fr_wrist_1 * F;
    DATA(WR2, ELBOW) = F(AZ);
    DATA(ELBOW, WR2) = DATA(WR2, ELBOW);
    F = frcTransf -> fr_upper_arm_X_fr_forearm * F;
    DATA(WR2, SHOULDER_LIFT) = F(AZ);
    DATA(SHOULDER_LIFT, WR2) = DATA(WR2, SHOULDER_LIFT);
    F = frcTransf -> fr_shoulder_X_fr_upper_arm * F;
    DATA(WR2, SHOULDER_PAN) = F(AZ);
    DATA(SHOULDER_PAN, WR2) = DATA(WR2, SHOULDER_PAN);

    // Link wrist_1:
    iit::rbd::transformInertia<Scalar>(wrist_1_Ic, frcTransf -> fr_forearm_X_fr_wrist_1, Ic_spare);
    forearm_Ic += Ic_spare;

    F = wrist_1_Ic.col(AZ);
    DATA(WR1, WR1) = F(AZ);

    F = frcTransf -> fr_forearm_X_fr_wrist_1 * F;
    DATA(WR1, ELBOW) = F(AZ);
    DATA(ELBOW, WR1) = DATA(WR1, ELBOW);
    F = frcTransf -> fr_upper_arm_X_fr_forearm * F;
    DATA(WR1, SHOULDER_LIFT) = F(AZ);
    DATA(SHOULDER_LIFT, WR1) = DATA(WR1, SHOULDER_LIFT);
    F = frcTransf -> fr_shoulder_X_fr_upper_arm * F;
    DATA(WR1, SHOULDER_PAN) = F(AZ);
    DATA(SHOULDER_PAN, WR1) = DATA(WR1, SHOULDER_PAN);

    // Link forearm:
    iit::rbd::transformInertia<Scalar>(forearm_Ic, frcTransf -> fr_upper_arm_X_fr_forearm, Ic_spare);
    upper_arm_Ic += Ic_spare;

    F = forearm_Ic.col(AZ);
    DATA(ELBOW, ELBOW) = F(AZ);

    F = frcTransf -> fr_upper_arm_X_fr_forearm * F;
    DATA(ELBOW, SHOULDER_LIFT) = F(AZ);
    DATA(SHOULDER_LIFT, ELBOW) = DATA(ELBOW, SHOULDER_LIFT);
    F = frcTransf -> fr_shoulder_X_fr_upper_arm * F;
    DATA(ELBOW, SHOULDER_PAN) = F(AZ);
    DATA(SHOULDER_PAN, ELBOW) = DATA(ELBOW, SHOULDER_PAN);

    // Link upper_arm:
    iit::rbd::transformInertia<Scalar>(upper_arm_Ic, frcTransf -> fr_shoulder_X_fr_upper_arm, Ic_spare);
    shoulder_Ic += Ic_spare;

    F = upper_arm_Ic.col(AZ);
    DATA(SHOULDER_LIFT, SHOULDER_LIFT) = F(AZ);

    F = frcTransf -> fr_shoulder_X_fr_upper_arm * F;
    DATA(SHOULDER_LIFT, SHOULDER_PAN) = F(AZ);
    DATA(SHOULDER_PAN, SHOULDER_LIFT) = DATA(SHOULDER_LIFT, SHOULDER_PAN);

    // Link shoulder:

    F = shoulder_Ic.col(AZ);
    DATA(SHOULDER_PAN, SHOULDER_PAN) = F(AZ);


    return *this;
}

#undef DATA
#undef F

void ur5::rcg::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint wr3, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(2, 0) = L(2, 0) - L(5, 2) * L(5, 0);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    L(1, 0) = L(1, 0) - L(5, 1) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint wr2, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint wr1, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint elbow, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint shoulder_lift, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint shoulder_pan, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void ur5::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 0) * Linv(2, 0)) + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 0) * Linv(1, 0)) + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
}

void ur5::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 1) * L(1, 0)) + (Linv(5, 2) * L(2, 0)) + (Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
}
