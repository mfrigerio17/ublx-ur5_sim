#ifndef UR5_TRANSFORMS_H_
#define UR5_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace ur5 {
namespace rcg {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_shoulder_pan_X_fr_shoulder : public TransformMotion<Type_fr_shoulder_pan_X_fr_shoulder>
    {
        Type_fr_shoulder_pan_X_fr_shoulder();
        const Type_fr_shoulder_pan_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_lift_X_fr_upper_arm : public TransformMotion<Type_fr_shoulder_lift_X_fr_upper_arm>
    {
        Type_fr_shoulder_lift_X_fr_upper_arm();
        const Type_fr_shoulder_lift_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_elbow_X_fr_forearm : public TransformMotion<Type_fr_elbow_X_fr_forearm>
    {
        Type_fr_elbow_X_fr_forearm();
        const Type_fr_elbow_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wr1_X_fr_wrist_1 : public TransformMotion<Type_fr_wr1_X_fr_wrist_1>
    {
        Type_fr_wr1_X_fr_wrist_1();
        const Type_fr_wr1_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wr2_X_fr_wrist_2 : public TransformMotion<Type_fr_wr2_X_fr_wrist_2>
    {
        Type_fr_wr2_X_fr_wrist_2();
        const Type_fr_wr2_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wr3_X_fr_wrist_3 : public TransformMotion<Type_fr_wr3_X_fr_wrist_3>
    {
        Type_fr_wr3_X_fr_wrist_3();
        const Type_fr_wr3_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wrist_3 : public TransformMotion<Type_fr_base_X_fr_wrist_3>
    {
        Type_fr_base_X_fr_wrist_3();
        const Type_fr_base_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_pan : public TransformMotion<Type_fr_base_X_fr_shoulder_pan>
    {
        Type_fr_base_X_fr_shoulder_pan();
        const Type_fr_base_X_fr_shoulder_pan& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_lift : public TransformMotion<Type_fr_base_X_fr_shoulder_lift>
    {
        Type_fr_base_X_fr_shoulder_lift();
        const Type_fr_base_X_fr_shoulder_lift& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow : public TransformMotion<Type_fr_base_X_fr_elbow>
    {
        Type_fr_base_X_fr_elbow();
        const Type_fr_base_X_fr_elbow& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr1 : public TransformMotion<Type_fr_base_X_fr_wr1>
    {
        Type_fr_base_X_fr_wr1();
        const Type_fr_base_X_fr_wr1& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr2 : public TransformMotion<Type_fr_base_X_fr_wr2>
    {
        Type_fr_base_X_fr_wr2();
        const Type_fr_base_X_fr_wr2& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr3 : public TransformMotion<Type_fr_base_X_fr_wr3>
    {
        Type_fr_base_X_fr_wr3();
        const Type_fr_base_X_fr_wr3& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_base : public TransformMotion<Type_fr_shoulder_X_fr_base>
    {
        Type_fr_shoulder_X_fr_base();
        const Type_fr_shoulder_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder : public TransformMotion<Type_fr_base_X_fr_shoulder>
    {
        Type_fr_base_X_fr_shoulder();
        const Type_fr_base_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_shoulder : public TransformMotion<Type_fr_upper_arm_X_fr_shoulder>
    {
        Type_fr_upper_arm_X_fr_shoulder();
        const Type_fr_upper_arm_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_upper_arm : public TransformMotion<Type_fr_shoulder_X_fr_upper_arm>
    {
        Type_fr_shoulder_X_fr_upper_arm();
        const Type_fr_shoulder_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_upper_arm : public TransformMotion<Type_fr_forearm_X_fr_upper_arm>
    {
        Type_fr_forearm_X_fr_upper_arm();
        const Type_fr_forearm_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_forearm : public TransformMotion<Type_fr_upper_arm_X_fr_forearm>
    {
        Type_fr_upper_arm_X_fr_forearm();
        const Type_fr_upper_arm_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_forearm : public TransformMotion<Type_fr_wrist_1_X_fr_forearm>
    {
        Type_fr_wrist_1_X_fr_forearm();
        const Type_fr_wrist_1_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_wrist_1 : public TransformMotion<Type_fr_forearm_X_fr_wrist_1>
    {
        Type_fr_forearm_X_fr_wrist_1();
        const Type_fr_forearm_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_1 : public TransformMotion<Type_fr_wrist_2_X_fr_wrist_1>
    {
        Type_fr_wrist_2_X_fr_wrist_1();
        const Type_fr_wrist_2_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_wrist_2 : public TransformMotion<Type_fr_wrist_1_X_fr_wrist_2>
    {
        Type_fr_wrist_1_X_fr_wrist_2();
        const Type_fr_wrist_1_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_3_X_fr_wrist_2 : public TransformMotion<Type_fr_wrist_3_X_fr_wrist_2>
    {
        Type_fr_wrist_3_X_fr_wrist_2();
        const Type_fr_wrist_3_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_3 : public TransformMotion<Type_fr_wrist_2_X_fr_wrist_3>
    {
        Type_fr_wrist_2_X_fr_wrist_3();
        const Type_fr_wrist_2_X_fr_wrist_3& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_shoulder_pan_X_fr_shoulder fr_shoulder_pan_X_fr_shoulder;
    Type_fr_shoulder_lift_X_fr_upper_arm fr_shoulder_lift_X_fr_upper_arm;
    Type_fr_elbow_X_fr_forearm fr_elbow_X_fr_forearm;
    Type_fr_wr1_X_fr_wrist_1 fr_wr1_X_fr_wrist_1;
    Type_fr_wr2_X_fr_wrist_2 fr_wr2_X_fr_wrist_2;
    Type_fr_wr3_X_fr_wrist_3 fr_wr3_X_fr_wrist_3;
    Type_fr_base_X_fr_wrist_3 fr_base_X_fr_wrist_3;
    Type_fr_base_X_fr_shoulder_pan fr_base_X_fr_shoulder_pan;
    Type_fr_base_X_fr_shoulder_lift fr_base_X_fr_shoulder_lift;
    Type_fr_base_X_fr_elbow fr_base_X_fr_elbow;
    Type_fr_base_X_fr_wr1 fr_base_X_fr_wr1;
    Type_fr_base_X_fr_wr2 fr_base_X_fr_wr2;
    Type_fr_base_X_fr_wr3 fr_base_X_fr_wr3;
    Type_fr_shoulder_X_fr_base fr_shoulder_X_fr_base;
    Type_fr_base_X_fr_shoulder fr_base_X_fr_shoulder;
    Type_fr_upper_arm_X_fr_shoulder fr_upper_arm_X_fr_shoulder;
    Type_fr_shoulder_X_fr_upper_arm fr_shoulder_X_fr_upper_arm;
    Type_fr_forearm_X_fr_upper_arm fr_forearm_X_fr_upper_arm;
    Type_fr_upper_arm_X_fr_forearm fr_upper_arm_X_fr_forearm;
    Type_fr_wrist_1_X_fr_forearm fr_wrist_1_X_fr_forearm;
    Type_fr_forearm_X_fr_wrist_1 fr_forearm_X_fr_wrist_1;
    Type_fr_wrist_2_X_fr_wrist_1 fr_wrist_2_X_fr_wrist_1;
    Type_fr_wrist_1_X_fr_wrist_2 fr_wrist_1_X_fr_wrist_2;
    Type_fr_wrist_3_X_fr_wrist_2 fr_wrist_3_X_fr_wrist_2;
    Type_fr_wrist_2_X_fr_wrist_3 fr_wrist_2_X_fr_wrist_3;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_shoulder_pan_X_fr_shoulder : public TransformForce<Type_fr_shoulder_pan_X_fr_shoulder>
    {
        Type_fr_shoulder_pan_X_fr_shoulder();
        const Type_fr_shoulder_pan_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_lift_X_fr_upper_arm : public TransformForce<Type_fr_shoulder_lift_X_fr_upper_arm>
    {
        Type_fr_shoulder_lift_X_fr_upper_arm();
        const Type_fr_shoulder_lift_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_elbow_X_fr_forearm : public TransformForce<Type_fr_elbow_X_fr_forearm>
    {
        Type_fr_elbow_X_fr_forearm();
        const Type_fr_elbow_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wr1_X_fr_wrist_1 : public TransformForce<Type_fr_wr1_X_fr_wrist_1>
    {
        Type_fr_wr1_X_fr_wrist_1();
        const Type_fr_wr1_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wr2_X_fr_wrist_2 : public TransformForce<Type_fr_wr2_X_fr_wrist_2>
    {
        Type_fr_wr2_X_fr_wrist_2();
        const Type_fr_wr2_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wr3_X_fr_wrist_3 : public TransformForce<Type_fr_wr3_X_fr_wrist_3>
    {
        Type_fr_wr3_X_fr_wrist_3();
        const Type_fr_wr3_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wrist_3 : public TransformForce<Type_fr_base_X_fr_wrist_3>
    {
        Type_fr_base_X_fr_wrist_3();
        const Type_fr_base_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_pan : public TransformForce<Type_fr_base_X_fr_shoulder_pan>
    {
        Type_fr_base_X_fr_shoulder_pan();
        const Type_fr_base_X_fr_shoulder_pan& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_lift : public TransformForce<Type_fr_base_X_fr_shoulder_lift>
    {
        Type_fr_base_X_fr_shoulder_lift();
        const Type_fr_base_X_fr_shoulder_lift& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow : public TransformForce<Type_fr_base_X_fr_elbow>
    {
        Type_fr_base_X_fr_elbow();
        const Type_fr_base_X_fr_elbow& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr1 : public TransformForce<Type_fr_base_X_fr_wr1>
    {
        Type_fr_base_X_fr_wr1();
        const Type_fr_base_X_fr_wr1& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr2 : public TransformForce<Type_fr_base_X_fr_wr2>
    {
        Type_fr_base_X_fr_wr2();
        const Type_fr_base_X_fr_wr2& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr3 : public TransformForce<Type_fr_base_X_fr_wr3>
    {
        Type_fr_base_X_fr_wr3();
        const Type_fr_base_X_fr_wr3& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_base : public TransformForce<Type_fr_shoulder_X_fr_base>
    {
        Type_fr_shoulder_X_fr_base();
        const Type_fr_shoulder_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder : public TransformForce<Type_fr_base_X_fr_shoulder>
    {
        Type_fr_base_X_fr_shoulder();
        const Type_fr_base_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_shoulder : public TransformForce<Type_fr_upper_arm_X_fr_shoulder>
    {
        Type_fr_upper_arm_X_fr_shoulder();
        const Type_fr_upper_arm_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_upper_arm : public TransformForce<Type_fr_shoulder_X_fr_upper_arm>
    {
        Type_fr_shoulder_X_fr_upper_arm();
        const Type_fr_shoulder_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_upper_arm : public TransformForce<Type_fr_forearm_X_fr_upper_arm>
    {
        Type_fr_forearm_X_fr_upper_arm();
        const Type_fr_forearm_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_forearm : public TransformForce<Type_fr_upper_arm_X_fr_forearm>
    {
        Type_fr_upper_arm_X_fr_forearm();
        const Type_fr_upper_arm_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_forearm : public TransformForce<Type_fr_wrist_1_X_fr_forearm>
    {
        Type_fr_wrist_1_X_fr_forearm();
        const Type_fr_wrist_1_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_wrist_1 : public TransformForce<Type_fr_forearm_X_fr_wrist_1>
    {
        Type_fr_forearm_X_fr_wrist_1();
        const Type_fr_forearm_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_1 : public TransformForce<Type_fr_wrist_2_X_fr_wrist_1>
    {
        Type_fr_wrist_2_X_fr_wrist_1();
        const Type_fr_wrist_2_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_wrist_2 : public TransformForce<Type_fr_wrist_1_X_fr_wrist_2>
    {
        Type_fr_wrist_1_X_fr_wrist_2();
        const Type_fr_wrist_1_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_3_X_fr_wrist_2 : public TransformForce<Type_fr_wrist_3_X_fr_wrist_2>
    {
        Type_fr_wrist_3_X_fr_wrist_2();
        const Type_fr_wrist_3_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_3 : public TransformForce<Type_fr_wrist_2_X_fr_wrist_3>
    {
        Type_fr_wrist_2_X_fr_wrist_3();
        const Type_fr_wrist_2_X_fr_wrist_3& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_shoulder_pan_X_fr_shoulder fr_shoulder_pan_X_fr_shoulder;
    Type_fr_shoulder_lift_X_fr_upper_arm fr_shoulder_lift_X_fr_upper_arm;
    Type_fr_elbow_X_fr_forearm fr_elbow_X_fr_forearm;
    Type_fr_wr1_X_fr_wrist_1 fr_wr1_X_fr_wrist_1;
    Type_fr_wr2_X_fr_wrist_2 fr_wr2_X_fr_wrist_2;
    Type_fr_wr3_X_fr_wrist_3 fr_wr3_X_fr_wrist_3;
    Type_fr_base_X_fr_wrist_3 fr_base_X_fr_wrist_3;
    Type_fr_base_X_fr_shoulder_pan fr_base_X_fr_shoulder_pan;
    Type_fr_base_X_fr_shoulder_lift fr_base_X_fr_shoulder_lift;
    Type_fr_base_X_fr_elbow fr_base_X_fr_elbow;
    Type_fr_base_X_fr_wr1 fr_base_X_fr_wr1;
    Type_fr_base_X_fr_wr2 fr_base_X_fr_wr2;
    Type_fr_base_X_fr_wr3 fr_base_X_fr_wr3;
    Type_fr_shoulder_X_fr_base fr_shoulder_X_fr_base;
    Type_fr_base_X_fr_shoulder fr_base_X_fr_shoulder;
    Type_fr_upper_arm_X_fr_shoulder fr_upper_arm_X_fr_shoulder;
    Type_fr_shoulder_X_fr_upper_arm fr_shoulder_X_fr_upper_arm;
    Type_fr_forearm_X_fr_upper_arm fr_forearm_X_fr_upper_arm;
    Type_fr_upper_arm_X_fr_forearm fr_upper_arm_X_fr_forearm;
    Type_fr_wrist_1_X_fr_forearm fr_wrist_1_X_fr_forearm;
    Type_fr_forearm_X_fr_wrist_1 fr_forearm_X_fr_wrist_1;
    Type_fr_wrist_2_X_fr_wrist_1 fr_wrist_2_X_fr_wrist_1;
    Type_fr_wrist_1_X_fr_wrist_2 fr_wrist_1_X_fr_wrist_2;
    Type_fr_wrist_3_X_fr_wrist_2 fr_wrist_3_X_fr_wrist_2;
    Type_fr_wrist_2_X_fr_wrist_3 fr_wrist_2_X_fr_wrist_3;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_shoulder_pan_X_fr_shoulder : public TransformHomogeneous<Type_fr_shoulder_pan_X_fr_shoulder>
    {
        Type_fr_shoulder_pan_X_fr_shoulder();
        const Type_fr_shoulder_pan_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_lift_X_fr_upper_arm : public TransformHomogeneous<Type_fr_shoulder_lift_X_fr_upper_arm>
    {
        Type_fr_shoulder_lift_X_fr_upper_arm();
        const Type_fr_shoulder_lift_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_elbow_X_fr_forearm : public TransformHomogeneous<Type_fr_elbow_X_fr_forearm>
    {
        Type_fr_elbow_X_fr_forearm();
        const Type_fr_elbow_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wr1_X_fr_wrist_1 : public TransformHomogeneous<Type_fr_wr1_X_fr_wrist_1>
    {
        Type_fr_wr1_X_fr_wrist_1();
        const Type_fr_wr1_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wr2_X_fr_wrist_2 : public TransformHomogeneous<Type_fr_wr2_X_fr_wrist_2>
    {
        Type_fr_wr2_X_fr_wrist_2();
        const Type_fr_wr2_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wr3_X_fr_wrist_3 : public TransformHomogeneous<Type_fr_wr3_X_fr_wrist_3>
    {
        Type_fr_wr3_X_fr_wrist_3();
        const Type_fr_wr3_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wrist_3 : public TransformHomogeneous<Type_fr_base_X_fr_wrist_3>
    {
        Type_fr_base_X_fr_wrist_3();
        const Type_fr_base_X_fr_wrist_3& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_pan : public TransformHomogeneous<Type_fr_base_X_fr_shoulder_pan>
    {
        Type_fr_base_X_fr_shoulder_pan();
        const Type_fr_base_X_fr_shoulder_pan& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder_lift : public TransformHomogeneous<Type_fr_base_X_fr_shoulder_lift>
    {
        Type_fr_base_X_fr_shoulder_lift();
        const Type_fr_base_X_fr_shoulder_lift& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow : public TransformHomogeneous<Type_fr_base_X_fr_elbow>
    {
        Type_fr_base_X_fr_elbow();
        const Type_fr_base_X_fr_elbow& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr1 : public TransformHomogeneous<Type_fr_base_X_fr_wr1>
    {
        Type_fr_base_X_fr_wr1();
        const Type_fr_base_X_fr_wr1& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr2 : public TransformHomogeneous<Type_fr_base_X_fr_wr2>
    {
        Type_fr_base_X_fr_wr2();
        const Type_fr_base_X_fr_wr2& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_wr3 : public TransformHomogeneous<Type_fr_base_X_fr_wr3>
    {
        Type_fr_base_X_fr_wr3();
        const Type_fr_base_X_fr_wr3& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_base : public TransformHomogeneous<Type_fr_shoulder_X_fr_base>
    {
        Type_fr_shoulder_X_fr_base();
        const Type_fr_shoulder_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_shoulder : public TransformHomogeneous<Type_fr_base_X_fr_shoulder>
    {
        Type_fr_base_X_fr_shoulder();
        const Type_fr_base_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_shoulder : public TransformHomogeneous<Type_fr_upper_arm_X_fr_shoulder>
    {
        Type_fr_upper_arm_X_fr_shoulder();
        const Type_fr_upper_arm_X_fr_shoulder& update(const state_t&);
    };
    
    struct Type_fr_shoulder_X_fr_upper_arm : public TransformHomogeneous<Type_fr_shoulder_X_fr_upper_arm>
    {
        Type_fr_shoulder_X_fr_upper_arm();
        const Type_fr_shoulder_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_upper_arm : public TransformHomogeneous<Type_fr_forearm_X_fr_upper_arm>
    {
        Type_fr_forearm_X_fr_upper_arm();
        const Type_fr_forearm_X_fr_upper_arm& update(const state_t&);
    };
    
    struct Type_fr_upper_arm_X_fr_forearm : public TransformHomogeneous<Type_fr_upper_arm_X_fr_forearm>
    {
        Type_fr_upper_arm_X_fr_forearm();
        const Type_fr_upper_arm_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_forearm : public TransformHomogeneous<Type_fr_wrist_1_X_fr_forearm>
    {
        Type_fr_wrist_1_X_fr_forearm();
        const Type_fr_wrist_1_X_fr_forearm& update(const state_t&);
    };
    
    struct Type_fr_forearm_X_fr_wrist_1 : public TransformHomogeneous<Type_fr_forearm_X_fr_wrist_1>
    {
        Type_fr_forearm_X_fr_wrist_1();
        const Type_fr_forearm_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_1 : public TransformHomogeneous<Type_fr_wrist_2_X_fr_wrist_1>
    {
        Type_fr_wrist_2_X_fr_wrist_1();
        const Type_fr_wrist_2_X_fr_wrist_1& update(const state_t&);
    };
    
    struct Type_fr_wrist_1_X_fr_wrist_2 : public TransformHomogeneous<Type_fr_wrist_1_X_fr_wrist_2>
    {
        Type_fr_wrist_1_X_fr_wrist_2();
        const Type_fr_wrist_1_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_3_X_fr_wrist_2 : public TransformHomogeneous<Type_fr_wrist_3_X_fr_wrist_2>
    {
        Type_fr_wrist_3_X_fr_wrist_2();
        const Type_fr_wrist_3_X_fr_wrist_2& update(const state_t&);
    };
    
    struct Type_fr_wrist_2_X_fr_wrist_3 : public TransformHomogeneous<Type_fr_wrist_2_X_fr_wrist_3>
    {
        Type_fr_wrist_2_X_fr_wrist_3();
        const Type_fr_wrist_2_X_fr_wrist_3& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_shoulder_pan_X_fr_shoulder fr_shoulder_pan_X_fr_shoulder;
    Type_fr_shoulder_lift_X_fr_upper_arm fr_shoulder_lift_X_fr_upper_arm;
    Type_fr_elbow_X_fr_forearm fr_elbow_X_fr_forearm;
    Type_fr_wr1_X_fr_wrist_1 fr_wr1_X_fr_wrist_1;
    Type_fr_wr2_X_fr_wrist_2 fr_wr2_X_fr_wrist_2;
    Type_fr_wr3_X_fr_wrist_3 fr_wr3_X_fr_wrist_3;
    Type_fr_base_X_fr_wrist_3 fr_base_X_fr_wrist_3;
    Type_fr_base_X_fr_shoulder_pan fr_base_X_fr_shoulder_pan;
    Type_fr_base_X_fr_shoulder_lift fr_base_X_fr_shoulder_lift;
    Type_fr_base_X_fr_elbow fr_base_X_fr_elbow;
    Type_fr_base_X_fr_wr1 fr_base_X_fr_wr1;
    Type_fr_base_X_fr_wr2 fr_base_X_fr_wr2;
    Type_fr_base_X_fr_wr3 fr_base_X_fr_wr3;
    Type_fr_shoulder_X_fr_base fr_shoulder_X_fr_base;
    Type_fr_base_X_fr_shoulder fr_base_X_fr_shoulder;
    Type_fr_upper_arm_X_fr_shoulder fr_upper_arm_X_fr_shoulder;
    Type_fr_shoulder_X_fr_upper_arm fr_shoulder_X_fr_upper_arm;
    Type_fr_forearm_X_fr_upper_arm fr_forearm_X_fr_upper_arm;
    Type_fr_upper_arm_X_fr_forearm fr_upper_arm_X_fr_forearm;
    Type_fr_wrist_1_X_fr_forearm fr_wrist_1_X_fr_forearm;
    Type_fr_forearm_X_fr_wrist_1 fr_forearm_X_fr_wrist_1;
    Type_fr_wrist_2_X_fr_wrist_1 fr_wrist_2_X_fr_wrist_1;
    Type_fr_wrist_1_X_fr_wrist_2 fr_wrist_1_X_fr_wrist_2;
    Type_fr_wrist_3_X_fr_wrist_2 fr_wrist_3_X_fr_wrist_2;
    Type_fr_wrist_2_X_fr_wrist_3 fr_wrist_2_X_fr_wrist_3;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
