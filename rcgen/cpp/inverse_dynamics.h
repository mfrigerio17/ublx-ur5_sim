#ifndef RCG_UR5_INVERSE_DYNAMICS_H_
#define RCG_UR5_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace ur5 {
namespace rcg {

/**
 * The Inverse Dynamics routine for the robot ur5.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ur5, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JointState& jForces,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JointState& jForces, const JointState& q);
    void G_terms(JointState& jForces);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
    void C_terms(JointState& jForces, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Velocity& getVelocity_shoulder() const { return shoulder_v; }
    const Acceleration& getAcceleration_shoulder() const { return shoulder_a; }
    const Force& getForce_shoulder() const { return shoulder_f; }
    const Velocity& getVelocity_upper_arm() const { return upper_arm_v; }
    const Acceleration& getAcceleration_upper_arm() const { return upper_arm_a; }
    const Force& getForce_upper_arm() const { return upper_arm_f; }
    const Velocity& getVelocity_forearm() const { return forearm_v; }
    const Acceleration& getAcceleration_forearm() const { return forearm_a; }
    const Force& getForce_forearm() const { return forearm_f; }
    const Velocity& getVelocity_wrist_1() const { return wrist_1_v; }
    const Acceleration& getAcceleration_wrist_1() const { return wrist_1_a; }
    const Force& getForce_wrist_1() const { return wrist_1_f; }
    const Velocity& getVelocity_wrist_2() const { return wrist_2_v; }
    const Acceleration& getAcceleration_wrist_2() const { return wrist_2_a; }
    const Force& getForce_wrist_2() const { return wrist_2_f; }
    const Velocity& getVelocity_wrist_3() const { return wrist_3_v; }
    const Acceleration& getAcceleration_wrist_3() const { return wrist_3_a; }
    const Force& getForce_wrist_3() const { return wrist_3_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'shoulder' :
    const InertiaMatrix& shoulder_I;
    Velocity      shoulder_v;
    Acceleration  shoulder_a;
    Force         shoulder_f;
    // Link 'upper_arm' :
    const InertiaMatrix& upper_arm_I;
    Velocity      upper_arm_v;
    Acceleration  upper_arm_a;
    Force         upper_arm_f;
    // Link 'forearm' :
    const InertiaMatrix& forearm_I;
    Velocity      forearm_v;
    Acceleration  forearm_a;
    Force         forearm_f;
    // Link 'wrist_1' :
    const InertiaMatrix& wrist_1_I;
    Velocity      wrist_1_v;
    Acceleration  wrist_1_a;
    Force         wrist_1_f;
    // Link 'wrist_2' :
    const InertiaMatrix& wrist_2_I;
    Velocity      wrist_2_v;
    Acceleration  wrist_2_a;
    Force         wrist_2_f;
    // Link 'wrist_3' :
    const InertiaMatrix& wrist_3_I;
    Velocity      wrist_3_v;
    Acceleration  wrist_3_a;
    Force         wrist_3_f;


private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_shoulder_X_fr_base)(q);
    (xm->fr_upper_arm_X_fr_shoulder)(q);
    (xm->fr_forearm_X_fr_upper_arm)(q);
    (xm->fr_wrist_1_X_fr_forearm)(q);
    (xm->fr_wrist_2_X_fr_wrist_1)(q);
    (xm->fr_wrist_3_X_fr_wrist_2)(q);
}

inline void InverseDynamics::G_terms(JointState& jForces, const JointState& q)
{
    setJointStatus(q);
    G_terms(jForces);
}

inline void InverseDynamics::C_terms(JointState& jForces, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms(jForces, qd);
}

inline void InverseDynamics::id(
    JointState& jForces,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, qd, qdd, fext);
}

}
}

#endif
