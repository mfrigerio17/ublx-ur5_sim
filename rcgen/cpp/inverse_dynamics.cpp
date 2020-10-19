#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace ur5::rcg;

// Initialization of static-const data
const ur5::rcg::InverseDynamics::ExtForces
ur5::rcg::InverseDynamics::zeroExtForces(Force::Zero());

ur5::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    shoulder_I(inertiaProps->getTensor_shoulder() ),
    upper_arm_I(inertiaProps->getTensor_upper_arm() ),
    forearm_I(inertiaProps->getTensor_forearm() ),
    wrist_1_I(inertiaProps->getTensor_wrist_1() ),
    wrist_2_I(inertiaProps->getTensor_wrist_2() ),
    wrist_3_I(inertiaProps->getTensor_wrist_3() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ur5, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    shoulder_v.setZero();
    upper_arm_v.setZero();
    forearm_v.setZero();
    wrist_1_v.setZero();
    wrist_2_v.setZero();
    wrist_3_v.setZero();

    vcross.setZero();
}

void ur5::rcg::InverseDynamics::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void ur5::rcg::InverseDynamics::G_terms(JointState& jForces)
{
    // Link 'shoulder'
    shoulder_a = (xm->fr_shoulder_X_fr_base).col(iit::rbd::LZ) * ur5::rcg::g;
    shoulder_f = shoulder_I * shoulder_a;
    // Link 'upper_arm'
    upper_arm_a = (xm->fr_upper_arm_X_fr_shoulder) * shoulder_a;
    upper_arm_f = upper_arm_I * upper_arm_a;
    // Link 'forearm'
    forearm_a = (xm->fr_forearm_X_fr_upper_arm) * upper_arm_a;
    forearm_f = forearm_I * forearm_a;
    // Link 'wrist_1'
    wrist_1_a = (xm->fr_wrist_1_X_fr_forearm) * forearm_a;
    wrist_1_f = wrist_1_I * wrist_1_a;
    // Link 'wrist_2'
    wrist_2_a = (xm->fr_wrist_2_X_fr_wrist_1) * wrist_1_a;
    wrist_2_f = wrist_2_I * wrist_2_a;
    // Link 'wrist_3'
    wrist_3_a = (xm->fr_wrist_3_X_fr_wrist_2) * wrist_2_a;
    wrist_3_f = wrist_3_I * wrist_3_a;

    secondPass(jForces);
}

void ur5::rcg::InverseDynamics::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'shoulder'
    shoulder_v(iit::rbd::AZ) = qd(SHOULDER_PAN);   // shoulder_v = vJ, for the first link of a fixed base robot
    
    shoulder_f = vxIv(qd(SHOULDER_PAN), shoulder_I);
    
    // Link 'upper_arm'
    upper_arm_v = ((xm->fr_upper_arm_X_fr_shoulder) * shoulder_v);
    upper_arm_v(iit::rbd::AZ) += qd(SHOULDER_LIFT);
    
    motionCrossProductMx<Scalar>(upper_arm_v, vcross);
    
    upper_arm_a = (vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT));
    
    upper_arm_f = upper_arm_I * upper_arm_a + vxIv(upper_arm_v, upper_arm_I);
    
    // Link 'forearm'
    forearm_v = ((xm->fr_forearm_X_fr_upper_arm) * upper_arm_v);
    forearm_v(iit::rbd::AZ) += qd(ELBOW);
    
    motionCrossProductMx<Scalar>(forearm_v, vcross);
    
    forearm_a = (xm->fr_forearm_X_fr_upper_arm) * upper_arm_a + vcross.col(iit::rbd::AZ) * qd(ELBOW);
    
    forearm_f = forearm_I * forearm_a + vxIv(forearm_v, forearm_I);
    
    // Link 'wrist_1'
    wrist_1_v = ((xm->fr_wrist_1_X_fr_forearm) * forearm_v);
    wrist_1_v(iit::rbd::AZ) += qd(WR1);
    
    motionCrossProductMx<Scalar>(wrist_1_v, vcross);
    
    wrist_1_a = (xm->fr_wrist_1_X_fr_forearm) * forearm_a + vcross.col(iit::rbd::AZ) * qd(WR1);
    
    wrist_1_f = wrist_1_I * wrist_1_a + vxIv(wrist_1_v, wrist_1_I);
    
    // Link 'wrist_2'
    wrist_2_v = ((xm->fr_wrist_2_X_fr_wrist_1) * wrist_1_v);
    wrist_2_v(iit::rbd::AZ) += qd(WR2);
    
    motionCrossProductMx<Scalar>(wrist_2_v, vcross);
    
    wrist_2_a = (xm->fr_wrist_2_X_fr_wrist_1) * wrist_1_a + vcross.col(iit::rbd::AZ) * qd(WR2);
    
    wrist_2_f = wrist_2_I * wrist_2_a + vxIv(wrist_2_v, wrist_2_I);
    
    // Link 'wrist_3'
    wrist_3_v = ((xm->fr_wrist_3_X_fr_wrist_2) * wrist_2_v);
    wrist_3_v(iit::rbd::AZ) += qd(WR3);
    
    motionCrossProductMx<Scalar>(wrist_3_v, vcross);
    
    wrist_3_a = (xm->fr_wrist_3_X_fr_wrist_2) * wrist_2_a + vcross.col(iit::rbd::AZ) * qd(WR3);
    
    wrist_3_f = wrist_3_I * wrist_3_a + vxIv(wrist_3_v, wrist_3_I);
    

    secondPass(jForces);
}


void ur5::rcg::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'shoulder'
    shoulder_a = (xm->fr_shoulder_X_fr_base).col(iit::rbd::LZ) * ur5::rcg::g;
    shoulder_a(iit::rbd::AZ) += qdd(SHOULDER_PAN);
    shoulder_v(iit::rbd::AZ) = qd(SHOULDER_PAN);   // shoulder_v = vJ, for the first link of a fixed base robot
    
    shoulder_f = shoulder_I * shoulder_a + vxIv(qd(SHOULDER_PAN), shoulder_I)  - fext[SHOULDER];
    
    // First pass, link 'upper_arm'
    upper_arm_v = ((xm->fr_upper_arm_X_fr_shoulder) * shoulder_v);
    upper_arm_v(iit::rbd::AZ) += qd(SHOULDER_LIFT);
    
    motionCrossProductMx<Scalar>(upper_arm_v, vcross);
    
    upper_arm_a = (xm->fr_upper_arm_X_fr_shoulder) * shoulder_a + vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT);
    upper_arm_a(iit::rbd::AZ) += qdd(SHOULDER_LIFT);
    
    upper_arm_f = upper_arm_I * upper_arm_a + vxIv(upper_arm_v, upper_arm_I) - fext[UPPER_ARM];
    
    // First pass, link 'forearm'
    forearm_v = ((xm->fr_forearm_X_fr_upper_arm) * upper_arm_v);
    forearm_v(iit::rbd::AZ) += qd(ELBOW);
    
    motionCrossProductMx<Scalar>(forearm_v, vcross);
    
    forearm_a = (xm->fr_forearm_X_fr_upper_arm) * upper_arm_a + vcross.col(iit::rbd::AZ) * qd(ELBOW);
    forearm_a(iit::rbd::AZ) += qdd(ELBOW);
    
    forearm_f = forearm_I * forearm_a + vxIv(forearm_v, forearm_I) - fext[FOREARM];
    
    // First pass, link 'wrist_1'
    wrist_1_v = ((xm->fr_wrist_1_X_fr_forearm) * forearm_v);
    wrist_1_v(iit::rbd::AZ) += qd(WR1);
    
    motionCrossProductMx<Scalar>(wrist_1_v, vcross);
    
    wrist_1_a = (xm->fr_wrist_1_X_fr_forearm) * forearm_a + vcross.col(iit::rbd::AZ) * qd(WR1);
    wrist_1_a(iit::rbd::AZ) += qdd(WR1);
    
    wrist_1_f = wrist_1_I * wrist_1_a + vxIv(wrist_1_v, wrist_1_I) - fext[WRIST_1];
    
    // First pass, link 'wrist_2'
    wrist_2_v = ((xm->fr_wrist_2_X_fr_wrist_1) * wrist_1_v);
    wrist_2_v(iit::rbd::AZ) += qd(WR2);
    
    motionCrossProductMx<Scalar>(wrist_2_v, vcross);
    
    wrist_2_a = (xm->fr_wrist_2_X_fr_wrist_1) * wrist_1_a + vcross.col(iit::rbd::AZ) * qd(WR2);
    wrist_2_a(iit::rbd::AZ) += qdd(WR2);
    
    wrist_2_f = wrist_2_I * wrist_2_a + vxIv(wrist_2_v, wrist_2_I) - fext[WRIST_2];
    
    // First pass, link 'wrist_3'
    wrist_3_v = ((xm->fr_wrist_3_X_fr_wrist_2) * wrist_2_v);
    wrist_3_v(iit::rbd::AZ) += qd(WR3);
    
    motionCrossProductMx<Scalar>(wrist_3_v, vcross);
    
    wrist_3_a = (xm->fr_wrist_3_X_fr_wrist_2) * wrist_2_a + vcross.col(iit::rbd::AZ) * qd(WR3);
    wrist_3_a(iit::rbd::AZ) += qdd(WR3);
    
    wrist_3_f = wrist_3_I * wrist_3_a + vxIv(wrist_3_v, wrist_3_I) - fext[WRIST_3];
    
}

void ur5::rcg::InverseDynamics::secondPass(JointState& jForces)
{
    // Link 'wrist_3'
    jForces(WR3) = wrist_3_f(iit::rbd::AZ);
    wrist_2_f += xm->fr_wrist_3_X_fr_wrist_2.transpose() * wrist_3_f;
    // Link 'wrist_2'
    jForces(WR2) = wrist_2_f(iit::rbd::AZ);
    wrist_1_f += xm->fr_wrist_2_X_fr_wrist_1.transpose() * wrist_2_f;
    // Link 'wrist_1'
    jForces(WR1) = wrist_1_f(iit::rbd::AZ);
    forearm_f += xm->fr_wrist_1_X_fr_forearm.transpose() * wrist_1_f;
    // Link 'forearm'
    jForces(ELBOW) = forearm_f(iit::rbd::AZ);
    upper_arm_f += xm->fr_forearm_X_fr_upper_arm.transpose() * forearm_f;
    // Link 'upper_arm'
    jForces(SHOULDER_LIFT) = upper_arm_f(iit::rbd::AZ);
    shoulder_f += xm->fr_upper_arm_X_fr_shoulder.transpose() * upper_arm_f;
    // Link 'shoulder'
    jForces(SHOULDER_PAN) = shoulder_f(iit::rbd::AZ);
}
