#ifndef RCG_UR5_FORWARD_DYNAMICS_H_
#define RCG_UR5_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace ur5 {
namespace rcg {

/**
 * The Forward Dynamics routine for the robot ur5.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ur5, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'shoulder' :
    Matrix66 shoulder_AI;
    Velocity shoulder_a;
    Velocity shoulder_v;
    Velocity shoulder_c;
    Force    shoulder_p;

    Column6 shoulder_U;
    Scalar shoulder_D;
    Scalar shoulder_u;
    // Link 'upper_arm' :
    Matrix66 upper_arm_AI;
    Velocity upper_arm_a;
    Velocity upper_arm_v;
    Velocity upper_arm_c;
    Force    upper_arm_p;

    Column6 upper_arm_U;
    Scalar upper_arm_D;
    Scalar upper_arm_u;
    // Link 'forearm' :
    Matrix66 forearm_AI;
    Velocity forearm_a;
    Velocity forearm_v;
    Velocity forearm_c;
    Force    forearm_p;

    Column6 forearm_U;
    Scalar forearm_D;
    Scalar forearm_u;
    // Link 'wrist_1' :
    Matrix66 wrist_1_AI;
    Velocity wrist_1_a;
    Velocity wrist_1_v;
    Velocity wrist_1_c;
    Force    wrist_1_p;

    Column6 wrist_1_U;
    Scalar wrist_1_D;
    Scalar wrist_1_u;
    // Link 'wrist_2' :
    Matrix66 wrist_2_AI;
    Velocity wrist_2_a;
    Velocity wrist_2_v;
    Velocity wrist_2_c;
    Force    wrist_2_p;

    Column6 wrist_2_U;
    Scalar wrist_2_D;
    Scalar wrist_2_u;
    // Link 'wrist_3' :
    Matrix66 wrist_3_AI;
    Velocity wrist_3_a;
    Velocity wrist_3_v;
    Velocity wrist_3_c;
    Force    wrist_3_p;

    Column6 wrist_3_U;
    Scalar wrist_3_D;
    Scalar wrist_3_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_shoulder_X_fr_base)(q);
    (motionTransforms-> fr_upper_arm_X_fr_shoulder)(q);
    (motionTransforms-> fr_forearm_X_fr_upper_arm)(q);
    (motionTransforms-> fr_wrist_1_X_fr_forearm)(q);
    (motionTransforms-> fr_wrist_2_X_fr_wrist_1)(q);
    (motionTransforms-> fr_wrist_3_X_fr_wrist_2)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}
}

#endif
