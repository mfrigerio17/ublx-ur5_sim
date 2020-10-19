#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const ur5::rcg::ForwardDynamics::ExtForces
    ur5::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

ur5::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    shoulder_v.setZero();
    shoulder_c.setZero();
    upper_arm_v.setZero();
    upper_arm_c.setZero();
    forearm_v.setZero();
    forearm_c.setZero();
    wrist_1_v.setZero();
    wrist_1_c.setZero();
    wrist_2_v.setZero();
    wrist_2_c.setZero();
    wrist_3_v.setZero();
    wrist_3_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void ur5::rcg::ForwardDynamics::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    shoulder_AI = inertiaProps->getTensor_shoulder();
    shoulder_p = - fext[SHOULDER];
    upper_arm_AI = inertiaProps->getTensor_upper_arm();
    upper_arm_p = - fext[UPPER_ARM];
    forearm_AI = inertiaProps->getTensor_forearm();
    forearm_p = - fext[FOREARM];
    wrist_1_AI = inertiaProps->getTensor_wrist_1();
    wrist_1_p = - fext[WRIST_1];
    wrist_2_AI = inertiaProps->getTensor_wrist_2();
    wrist_2_p = - fext[WRIST_2];
    wrist_3_AI = inertiaProps->getTensor_wrist_3();
    wrist_3_p = - fext[WRIST_3];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link shoulder
    //  - The spatial velocity:
    shoulder_v(AZ) = qd(SHOULDER_PAN);
    
    //  - The bias force term:
    shoulder_p += vxIv(qd(SHOULDER_PAN), shoulder_AI);
    
    // + Link upper_arm
    //  - The spatial velocity:
    upper_arm_v = (motionTransforms-> fr_upper_arm_X_fr_shoulder) * shoulder_v;
    upper_arm_v(AZ) += qd(SHOULDER_LIFT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(upper_arm_v, vcross);
    upper_arm_c = vcross.col(AZ) * qd(SHOULDER_LIFT);
    
    //  - The bias force term:
    upper_arm_p += vxIv(upper_arm_v, upper_arm_AI);
    
    // + Link forearm
    //  - The spatial velocity:
    forearm_v = (motionTransforms-> fr_forearm_X_fr_upper_arm) * upper_arm_v;
    forearm_v(AZ) += qd(ELBOW);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(forearm_v, vcross);
    forearm_c = vcross.col(AZ) * qd(ELBOW);
    
    //  - The bias force term:
    forearm_p += vxIv(forearm_v, forearm_AI);
    
    // + Link wrist_1
    //  - The spatial velocity:
    wrist_1_v = (motionTransforms-> fr_wrist_1_X_fr_forearm) * forearm_v;
    wrist_1_v(AZ) += qd(WR1);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(wrist_1_v, vcross);
    wrist_1_c = vcross.col(AZ) * qd(WR1);
    
    //  - The bias force term:
    wrist_1_p += vxIv(wrist_1_v, wrist_1_AI);
    
    // + Link wrist_2
    //  - The spatial velocity:
    wrist_2_v = (motionTransforms-> fr_wrist_2_X_fr_wrist_1) * wrist_1_v;
    wrist_2_v(AZ) += qd(WR2);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(wrist_2_v, vcross);
    wrist_2_c = vcross.col(AZ) * qd(WR2);
    
    //  - The bias force term:
    wrist_2_p += vxIv(wrist_2_v, wrist_2_AI);
    
    // + Link wrist_3
    //  - The spatial velocity:
    wrist_3_v = (motionTransforms-> fr_wrist_3_X_fr_wrist_2) * wrist_2_v;
    wrist_3_v(AZ) += qd(WR3);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(wrist_3_v, vcross);
    wrist_3_c = vcross.col(AZ) * qd(WR3);
    
    //  - The bias force term:
    wrist_3_p += vxIv(wrist_3_v, wrist_3_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link wrist_3
    wrist_3_u = tau(WR3) - wrist_3_p(AZ);
    wrist_3_U = wrist_3_AI.col(AZ);
    wrist_3_D = wrist_3_U(AZ);
    
    compute_Ia_revolute(wrist_3_AI, wrist_3_U, wrist_3_D, Ia_r);  // same as: Ia_r = wrist_3_AI - wrist_3_U/wrist_3_D * wrist_3_U.transpose();
    pa = wrist_3_p + Ia_r * wrist_3_c + wrist_3_U * wrist_3_u/wrist_3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_3_X_fr_wrist_2, IaB);
    wrist_2_AI += IaB;
    wrist_2_p += (motionTransforms-> fr_wrist_3_X_fr_wrist_2).transpose() * pa;
    
    // + Link wrist_2
    wrist_2_u = tau(WR2) - wrist_2_p(AZ);
    wrist_2_U = wrist_2_AI.col(AZ);
    wrist_2_D = wrist_2_U(AZ);
    
    compute_Ia_revolute(wrist_2_AI, wrist_2_U, wrist_2_D, Ia_r);  // same as: Ia_r = wrist_2_AI - wrist_2_U/wrist_2_D * wrist_2_U.transpose();
    pa = wrist_2_p + Ia_r * wrist_2_c + wrist_2_U * wrist_2_u/wrist_2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_2_X_fr_wrist_1, IaB);
    wrist_1_AI += IaB;
    wrist_1_p += (motionTransforms-> fr_wrist_2_X_fr_wrist_1).transpose() * pa;
    
    // + Link wrist_1
    wrist_1_u = tau(WR1) - wrist_1_p(AZ);
    wrist_1_U = wrist_1_AI.col(AZ);
    wrist_1_D = wrist_1_U(AZ);
    
    compute_Ia_revolute(wrist_1_AI, wrist_1_U, wrist_1_D, Ia_r);  // same as: Ia_r = wrist_1_AI - wrist_1_U/wrist_1_D * wrist_1_U.transpose();
    pa = wrist_1_p + Ia_r * wrist_1_c + wrist_1_U * wrist_1_u/wrist_1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_1_X_fr_forearm, IaB);
    forearm_AI += IaB;
    forearm_p += (motionTransforms-> fr_wrist_1_X_fr_forearm).transpose() * pa;
    
    // + Link forearm
    forearm_u = tau(ELBOW) - forearm_p(AZ);
    forearm_U = forearm_AI.col(AZ);
    forearm_D = forearm_U(AZ);
    
    compute_Ia_revolute(forearm_AI, forearm_U, forearm_D, Ia_r);  // same as: Ia_r = forearm_AI - forearm_U/forearm_D * forearm_U.transpose();
    pa = forearm_p + Ia_r * forearm_c + forearm_U * forearm_u/forearm_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_forearm_X_fr_upper_arm, IaB);
    upper_arm_AI += IaB;
    upper_arm_p += (motionTransforms-> fr_forearm_X_fr_upper_arm).transpose() * pa;
    
    // + Link upper_arm
    upper_arm_u = tau(SHOULDER_LIFT) - upper_arm_p(AZ);
    upper_arm_U = upper_arm_AI.col(AZ);
    upper_arm_D = upper_arm_U(AZ);
    
    compute_Ia_revolute(upper_arm_AI, upper_arm_U, upper_arm_D, Ia_r);  // same as: Ia_r = upper_arm_AI - upper_arm_U/upper_arm_D * upper_arm_U.transpose();
    pa = upper_arm_p + Ia_r * upper_arm_c + upper_arm_U * upper_arm_u/upper_arm_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_upper_arm_X_fr_shoulder, IaB);
    shoulder_AI += IaB;
    shoulder_p += (motionTransforms-> fr_upper_arm_X_fr_shoulder).transpose() * pa;
    
    // + Link shoulder
    shoulder_u = tau(SHOULDER_PAN) - shoulder_p(AZ);
    shoulder_U = shoulder_AI.col(AZ);
    shoulder_D = shoulder_U(AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    shoulder_a = (motionTransforms-> fr_shoulder_X_fr_base).col(LZ) * (ur5::rcg::g);
    qdd(SHOULDER_PAN) = (shoulder_u - shoulder_U.dot(shoulder_a)) / shoulder_D;
    shoulder_a(AZ) += qdd(SHOULDER_PAN);
    
    upper_arm_a = (motionTransforms-> fr_upper_arm_X_fr_shoulder) * shoulder_a + upper_arm_c;
    qdd(SHOULDER_LIFT) = (upper_arm_u - upper_arm_U.dot(upper_arm_a)) / upper_arm_D;
    upper_arm_a(AZ) += qdd(SHOULDER_LIFT);
    
    forearm_a = (motionTransforms-> fr_forearm_X_fr_upper_arm) * upper_arm_a + forearm_c;
    qdd(ELBOW) = (forearm_u - forearm_U.dot(forearm_a)) / forearm_D;
    forearm_a(AZ) += qdd(ELBOW);
    
    wrist_1_a = (motionTransforms-> fr_wrist_1_X_fr_forearm) * forearm_a + wrist_1_c;
    qdd(WR1) = (wrist_1_u - wrist_1_U.dot(wrist_1_a)) / wrist_1_D;
    wrist_1_a(AZ) += qdd(WR1);
    
    wrist_2_a = (motionTransforms-> fr_wrist_2_X_fr_wrist_1) * wrist_1_a + wrist_2_c;
    qdd(WR2) = (wrist_2_u - wrist_2_U.dot(wrist_2_a)) / wrist_2_D;
    wrist_2_a(AZ) += qdd(WR2);
    
    wrist_3_a = (motionTransforms-> fr_wrist_3_X_fr_wrist_2) * wrist_2_a + wrist_3_c;
    qdd(WR3) = (wrist_3_u - wrist_3_U.dot(wrist_3_a)) / wrist_3_D;
    wrist_3_a(AZ) += qdd(WR3);
    
    
}
