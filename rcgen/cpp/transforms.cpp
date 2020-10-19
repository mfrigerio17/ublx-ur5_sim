#include "transforms.h"

using namespace ur5::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_shoulder_pan_X_fr_shoulder(),
    fr_shoulder_lift_X_fr_upper_arm(),
    fr_elbow_X_fr_forearm(),
    fr_wr1_X_fr_wrist_1(),
    fr_wr2_X_fr_wrist_2(),
    fr_wr3_X_fr_wrist_3(),
    fr_base_X_fr_wrist_3(),
    fr_base_X_fr_shoulder_pan(),
    fr_base_X_fr_shoulder_lift(),
    fr_base_X_fr_elbow(),
    fr_base_X_fr_wr1(),
    fr_base_X_fr_wr2(),
    fr_base_X_fr_wr3(),
    fr_shoulder_X_fr_base(),
    fr_base_X_fr_shoulder(),
    fr_upper_arm_X_fr_shoulder(),
    fr_shoulder_X_fr_upper_arm(),
    fr_forearm_X_fr_upper_arm(),
    fr_upper_arm_X_fr_forearm(),
    fr_wrist_1_X_fr_forearm(),
    fr_forearm_X_fr_wrist_1(),
    fr_wrist_2_X_fr_wrist_1(),
    fr_wrist_1_X_fr_wrist_2(),
    fr_wrist_3_X_fr_wrist_2(),
    fr_wrist_2_X_fr_wrist_3()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_shoulder_pan_X_fr_shoulder(),
    fr_shoulder_lift_X_fr_upper_arm(),
    fr_elbow_X_fr_forearm(),
    fr_wr1_X_fr_wrist_1(),
    fr_wr2_X_fr_wrist_2(),
    fr_wr3_X_fr_wrist_3(),
    fr_base_X_fr_wrist_3(),
    fr_base_X_fr_shoulder_pan(),
    fr_base_X_fr_shoulder_lift(),
    fr_base_X_fr_elbow(),
    fr_base_X_fr_wr1(),
    fr_base_X_fr_wr2(),
    fr_base_X_fr_wr3(),
    fr_shoulder_X_fr_base(),
    fr_base_X_fr_shoulder(),
    fr_upper_arm_X_fr_shoulder(),
    fr_shoulder_X_fr_upper_arm(),
    fr_forearm_X_fr_upper_arm(),
    fr_upper_arm_X_fr_forearm(),
    fr_wrist_1_X_fr_forearm(),
    fr_forearm_X_fr_wrist_1(),
    fr_wrist_2_X_fr_wrist_1(),
    fr_wrist_1_X_fr_wrist_2(),
    fr_wrist_3_X_fr_wrist_2(),
    fr_wrist_2_X_fr_wrist_3()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_shoulder_pan_X_fr_shoulder(),
    fr_shoulder_lift_X_fr_upper_arm(),
    fr_elbow_X_fr_forearm(),
    fr_wr1_X_fr_wrist_1(),
    fr_wr2_X_fr_wrist_2(),
    fr_wr3_X_fr_wrist_3(),
    fr_base_X_fr_wrist_3(),
    fr_base_X_fr_shoulder_pan(),
    fr_base_X_fr_shoulder_lift(),
    fr_base_X_fr_elbow(),
    fr_base_X_fr_wr1(),
    fr_base_X_fr_wr2(),
    fr_base_X_fr_wr3(),
    fr_shoulder_X_fr_base(),
    fr_base_X_fr_shoulder(),
    fr_upper_arm_X_fr_shoulder(),
    fr_shoulder_X_fr_upper_arm(),
    fr_forearm_X_fr_upper_arm(),
    fr_upper_arm_X_fr_forearm(),
    fr_wrist_1_X_fr_forearm(),
    fr_forearm_X_fr_wrist_1(),
    fr_wrist_2_X_fr_wrist_1(),
    fr_wrist_1_X_fr_wrist_2(),
    fr_wrist_3_X_fr_wrist_2(),
    fr_wrist_2_X_fr_wrist_3()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_shoulder_pan_X_fr_shoulder::Type_fr_shoulder_pan_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shoulder_pan_X_fr_shoulder& MotionTransforms::Type_fr_shoulder_pan_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = -sin_q_shoulder_pan;
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
MotionTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::Type_fr_shoulder_lift_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shoulder_lift_X_fr_upper_arm& MotionTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = cos_q_shoulder_lift;
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(1,0) = sin_q_shoulder_lift;
    (*this)(1,1) = cos_q_shoulder_lift;
    (*this)(3,3) = cos_q_shoulder_lift;
    (*this)(3,4) = -sin_q_shoulder_lift;
    (*this)(4,3) = sin_q_shoulder_lift;
    (*this)(4,4) = cos_q_shoulder_lift;
    return *this;
}
MotionTransforms::Type_fr_elbow_X_fr_forearm::Type_fr_elbow_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_elbow_X_fr_forearm& MotionTransforms::Type_fr_elbow_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = -sin_q_elbow;
    (*this)(4,3) = sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    return *this;
}
MotionTransforms::Type_fr_wr1_X_fr_wrist_1::Type_fr_wr1_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_wr1_X_fr_wrist_1& MotionTransforms::Type_fr_wr1_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = -sin_q_wr1;
    (*this)(4,3) = sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    return *this;
}
MotionTransforms::Type_fr_wr2_X_fr_wrist_2::Type_fr_wr2_X_fr_wrist_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_wr2_X_fr_wrist_2& MotionTransforms::Type_fr_wr2_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = cos_q_wr2;
    (*this)(0,1) = -sin_q_wr2;
    (*this)(1,0) = sin_q_wr2;
    (*this)(1,1) = cos_q_wr2;
    (*this)(3,3) = cos_q_wr2;
    (*this)(3,4) = -sin_q_wr2;
    (*this)(4,3) = sin_q_wr2;
    (*this)(4,4) = cos_q_wr2;
    return *this;
}
MotionTransforms::Type_fr_wr3_X_fr_wrist_3::Type_fr_wr3_X_fr_wrist_3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_wr3_X_fr_wrist_3& MotionTransforms::Type_fr_wr3_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = cos_q_wr3;
    (*this)(0,1) = -sin_q_wr3;
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(3,3) = cos_q_wr3;
    (*this)(3,4) = -sin_q_wr3;
    (*this)(4,3) = sin_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_wrist_3::Type_fr_base_X_fr_wrist_3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_wrist_3& MotionTransforms::Type_fr_base_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = (((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(0,1) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(1,0) = (((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(1,1) = (((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(2,0) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * sin_q_wr3)+(((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(2,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * cos_q_wr3);
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(3,0) = (((((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((- tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(3,1) = ((( tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1)) * sin_q_wr3)+(((((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(3,2) = ((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan)) * cos_q_wr2);
    (*this)(3,3) = (((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(3,4) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(3,5) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(4,0) = ((((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((- tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(4,1) = ((( tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(4,2) = ((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * sin_q_wr2)+(((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan)) * cos_q_wr2);
    (*this)(4,3) = (((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(4,4) = (((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(4,5) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(5,0) = ((((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(5,1) = ((((((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(5,2) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr2);
    (*this)(5,3) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * sin_q_wr3)+(((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(5,4) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * cos_q_wr3);
    (*this)(5,5) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_shoulder_pan::Type_fr_base_X_fr_shoulder_pan()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_shoulder_pan;    // Maxima DSL: -_k__tz_shoulder_pan
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_shoulder_pan;    // Maxima DSL: _k__tz_shoulder_pan
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_X_fr_shoulder_pan& MotionTransforms::Type_fr_base_X_fr_shoulder_pan::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_shoulder_lift::Type_fr_base_X_fr_shoulder_lift()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_shoulder_lift;    // Maxima DSL: -_k__tx_shoulder_lift
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_shoulder_lift& MotionTransforms::Type_fr_base_X_fr_shoulder_lift::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(3,0) =  tx_shoulder_lift * sin_q_shoulder_pan;
    (*this)(3,1) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(3,2) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(3,4) = sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,0) = - tx_shoulder_lift * cos_q_shoulder_pan;
    (*this)(4,1) =  tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(4,2) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(4,4) = -cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_elbow::Type_fr_base_X_fr_elbow()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_elbow& MotionTransforms::Type_fr_base_X_fr_elbow::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = sin_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,1) = cos_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(1,0) = -sin_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    (*this)(3,0) = (( tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_lift * sin_q_shoulder_pan)+( tz_shoulder_pan * sin_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(3,1) = ((- tz_elbow- tx_shoulder_lift) * sin_q_shoulder_lift * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_shoulder_lift)+ tx_elbow) * cos_q_shoulder_pan);
    (*this)(3,2) = ((- tx_elbow * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan;
    (*this)(3,3) = sin_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(3,4) = cos_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,0) = ( tz_shoulder_pan * sin_q_shoulder_lift * sin_q_shoulder_pan)+((- tz_elbow- tx_shoulder_lift) * cos_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(4,1) = ((( tz_shoulder_pan * cos_q_shoulder_lift)+ tx_elbow) * sin_q_shoulder_pan)+(( tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(4,2) = (( tx_elbow * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan;
    (*this)(4,3) = -sin_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(4,4) = -cos_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    (*this)(5,0) = (- tz_elbow- tx_shoulder_lift) * sin_q_shoulder_lift;
    (*this)(5,1) = (- tz_elbow- tx_shoulder_lift) * cos_q_shoulder_lift;
    (*this)(5,2) =  tx_elbow * sin_q_shoulder_lift;
    (*this)(5,3) = cos_q_shoulder_lift;
    (*this)(5,4) = -sin_q_shoulder_lift;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_wr1::Type_fr_base_X_fr_wr1()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_wr1& MotionTransforms::Type_fr_base_X_fr_wr1::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = ((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,1) = ((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(1,0) = ((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,1) = ((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(2,0) = (cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift);
    (*this)(2,1) = (-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift);
    (*this)(3,0) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan);
    (*this)(3,1) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan);
    (*this)(3,2) = (( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan;
    (*this)(3,3) = ((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(3,4) = ((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,0) = ((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(4,1) = (((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(4,2) = ((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan;
    (*this)(4,3) = ((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(4,4) = ((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    (*this)(5,0) = ((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(5,1) = (( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift);
    (*this)(5,2) = ((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(5,3) = (cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift);
    (*this)(5,4) = (-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_wr2::Type_fr_base_X_fr_wr2()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_wr2& MotionTransforms::Type_fr_base_X_fr_wr2::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,2) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,2) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(2,1) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,2) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(3,0) = ((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan);
    (*this)(3,1) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan);
    (*this)(3,2) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,5) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(4,0) = (((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan);
    (*this)(4,1) = ((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan);
    (*this)(4,2) = (((((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,5) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(5,0) = ((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(5,1) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,2) = (((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,4) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,5) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_wr3::Type_fr_base_X_fr_wr3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_wr3& MotionTransforms::Type_fr_base_X_fr_wr3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,1) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(1,0) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(2,0) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,1) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(3,0) = (- tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(3,1) = (((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,2) = ((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan)) * cos_q_wr2);
    (*this)(3,3) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,4) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2);
    (*this)(3,5) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(4,0) = (- tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(4,1) = ((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,2) = ((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * sin_q_wr2)+(((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan)) * cos_q_wr2);
    (*this)(4,3) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,4) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2);
    (*this)(4,5) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(5,0) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,1) = ((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,2) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr2);
    (*this)(5,3) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,4) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    (*this)(5,5) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    return *this;
}
MotionTransforms::Type_fr_shoulder_X_fr_base::Type_fr_shoulder_X_fr_base()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shoulder_X_fr_base& MotionTransforms::Type_fr_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(1,0) = -sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(3,0) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(3,1) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = sin_q_shoulder_pan;
    (*this)(4,0) = - tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(4,1) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(4,3) = -sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_shoulder::Type_fr_base_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_X_fr_shoulder& MotionTransforms::Type_fr_base_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(3,0) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(3,1) = - tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = -sin_q_shoulder_pan;
    (*this)(4,0) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(4,1) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
MotionTransforms::Type_fr_upper_arm_X_fr_shoulder::Type_fr_upper_arm_X_fr_shoulder()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_upper_arm_X_fr_shoulder& MotionTransforms::Type_fr_upper_arm_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(0,2) = cos_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(1,2) = -sin_q_shoulder_lift;
    (*this)(3,1) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(3,2) = - tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(3,4) = -sin_q_shoulder_lift;
    (*this)(3,5) = cos_q_shoulder_lift;
    (*this)(4,1) =  tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(4,2) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(4,4) = -cos_q_shoulder_lift;
    (*this)(4,5) = -sin_q_shoulder_lift;
    return *this;
}
MotionTransforms::Type_fr_shoulder_X_fr_upper_arm::Type_fr_shoulder_X_fr_upper_arm()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_shoulder_X_fr_upper_arm& MotionTransforms::Type_fr_shoulder_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(1,0) = -sin_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    (*this)(4,0) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(4,1) =  tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(4,3) = -sin_q_shoulder_lift;
    (*this)(4,4) = -cos_q_shoulder_lift;
    (*this)(5,0) = - tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(5,1) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(5,3) = cos_q_shoulder_lift;
    (*this)(5,4) = -sin_q_shoulder_lift;
    return *this;
}
MotionTransforms::Type_fr_forearm_X_fr_upper_arm::Type_fr_forearm_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_elbow;    // Maxima DSL: -_k__tx_elbow
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_forearm_X_fr_upper_arm& MotionTransforms::Type_fr_forearm_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = sin_q_elbow;
    (*this)(1,0) = -sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(3,0) = - tz_elbow * sin_q_elbow;
    (*this)(3,1) =  tz_elbow * cos_q_elbow;
    (*this)(3,2) =  tx_elbow * sin_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = sin_q_elbow;
    (*this)(4,0) = - tz_elbow * cos_q_elbow;
    (*this)(4,1) = - tz_elbow * sin_q_elbow;
    (*this)(4,2) =  tx_elbow * cos_q_elbow;
    (*this)(4,3) = -sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    return *this;
}
MotionTransforms::Type_fr_upper_arm_X_fr_forearm::Type_fr_upper_arm_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_elbow;    // Maxima DSL: -_k__tx_elbow
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_upper_arm_X_fr_forearm& MotionTransforms::Type_fr_upper_arm_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(3,0) = - tz_elbow * sin_q_elbow;
    (*this)(3,1) = - tz_elbow * cos_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = -sin_q_elbow;
    (*this)(4,0) =  tz_elbow * cos_q_elbow;
    (*this)(4,1) = - tz_elbow * sin_q_elbow;
    (*this)(4,3) = sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    (*this)(5,0) =  tx_elbow * sin_q_elbow;
    (*this)(5,1) =  tx_elbow * cos_q_elbow;
    return *this;
}
MotionTransforms::Type_fr_wrist_1_X_fr_forearm::Type_fr_wrist_1_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_wr1;    // Maxima DSL: -_k__tx_wr1
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_wrist_1_X_fr_forearm& MotionTransforms::Type_fr_wrist_1_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = sin_q_wr1;
    (*this)(1,0) = -sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(3,0) = - tz_wr1 * sin_q_wr1;
    (*this)(3,1) =  tz_wr1 * cos_q_wr1;
    (*this)(3,2) =  tx_wr1 * sin_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = sin_q_wr1;
    (*this)(4,0) = - tz_wr1 * cos_q_wr1;
    (*this)(4,1) = - tz_wr1 * sin_q_wr1;
    (*this)(4,2) =  tx_wr1 * cos_q_wr1;
    (*this)(4,3) = -sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    return *this;
}
MotionTransforms::Type_fr_forearm_X_fr_wrist_1::Type_fr_forearm_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_wr1;    // Maxima DSL: -_k__tx_wr1
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_forearm_X_fr_wrist_1& MotionTransforms::Type_fr_forearm_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(3,0) = - tz_wr1 * sin_q_wr1;
    (*this)(3,1) = - tz_wr1 * cos_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = -sin_q_wr1;
    (*this)(4,0) =  tz_wr1 * cos_q_wr1;
    (*this)(4,1) = - tz_wr1 * sin_q_wr1;
    (*this)(4,3) = sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    (*this)(5,0) =  tx_wr1 * sin_q_wr1;
    (*this)(5,1) =  tx_wr1 * cos_q_wr1;
    return *this;
}
MotionTransforms::Type_fr_wrist_2_X_fr_wrist_1::Type_fr_wrist_2_X_fr_wrist_1()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_2_X_fr_wrist_1& MotionTransforms::Type_fr_wrist_2_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,1) = -sin_q_wr2;
    (*this)(0,2) = cos_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(1,2) = -sin_q_wr2;
    (*this)(3,1) = - tx_wr2 * cos_q_wr2;
    (*this)(3,2) = - tx_wr2 * sin_q_wr2;
    (*this)(3,4) = -sin_q_wr2;
    (*this)(3,5) = cos_q_wr2;
    (*this)(4,1) =  tx_wr2 * sin_q_wr2;
    (*this)(4,2) = - tx_wr2 * cos_q_wr2;
    (*this)(4,4) = -cos_q_wr2;
    (*this)(4,5) = -sin_q_wr2;
    return *this;
}
MotionTransforms::Type_fr_wrist_1_X_fr_wrist_2::Type_fr_wrist_1_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_1_X_fr_wrist_2& MotionTransforms::Type_fr_wrist_1_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(1,0) = -sin_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(2,0) = cos_q_wr2;
    (*this)(2,1) = -sin_q_wr2;
    (*this)(4,0) = - tx_wr2 * cos_q_wr2;
    (*this)(4,1) =  tx_wr2 * sin_q_wr2;
    (*this)(4,3) = -sin_q_wr2;
    (*this)(4,4) = -cos_q_wr2;
    (*this)(5,0) = - tx_wr2 * sin_q_wr2;
    (*this)(5,1) = - tx_wr2 * cos_q_wr2;
    (*this)(5,3) = cos_q_wr2;
    (*this)(5,4) = -sin_q_wr2;
    return *this;
}
MotionTransforms::Type_fr_wrist_3_X_fr_wrist_2::Type_fr_wrist_3_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_3_X_fr_wrist_2& MotionTransforms::Type_fr_wrist_3_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,1) = sin_q_wr3;
    (*this)(0,2) = -cos_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(1,2) = sin_q_wr3;
    (*this)(3,1) =  tx_wr3 * cos_q_wr3;
    (*this)(3,2) =  tx_wr3 * sin_q_wr3;
    (*this)(3,4) = sin_q_wr3;
    (*this)(3,5) = -cos_q_wr3;
    (*this)(4,1) = - tx_wr3 * sin_q_wr3;
    (*this)(4,2) =  tx_wr3 * cos_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    (*this)(4,5) = sin_q_wr3;
    return *this;
}
MotionTransforms::Type_fr_wrist_2_X_fr_wrist_3::Type_fr_wrist_2_X_fr_wrist_3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_2_X_fr_wrist_3& MotionTransforms::Type_fr_wrist_2_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(2,0) = -cos_q_wr3;
    (*this)(2,1) = sin_q_wr3;
    (*this)(4,0) =  tx_wr3 * cos_q_wr3;
    (*this)(4,1) = - tx_wr3 * sin_q_wr3;
    (*this)(4,3) = sin_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    (*this)(5,0) =  tx_wr3 * sin_q_wr3;
    (*this)(5,1) =  tx_wr3 * cos_q_wr3;
    (*this)(5,3) = -cos_q_wr3;
    (*this)(5,4) = sin_q_wr3;
    return *this;
}

ForceTransforms::Type_fr_shoulder_pan_X_fr_shoulder::Type_fr_shoulder_pan_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shoulder_pan_X_fr_shoulder& ForceTransforms::Type_fr_shoulder_pan_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = -sin_q_shoulder_pan;
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
ForceTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::Type_fr_shoulder_lift_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shoulder_lift_X_fr_upper_arm& ForceTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = cos_q_shoulder_lift;
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(1,0) = sin_q_shoulder_lift;
    (*this)(1,1) = cos_q_shoulder_lift;
    (*this)(3,3) = cos_q_shoulder_lift;
    (*this)(3,4) = -sin_q_shoulder_lift;
    (*this)(4,3) = sin_q_shoulder_lift;
    (*this)(4,4) = cos_q_shoulder_lift;
    return *this;
}
ForceTransforms::Type_fr_elbow_X_fr_forearm::Type_fr_elbow_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_elbow_X_fr_forearm& ForceTransforms::Type_fr_elbow_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = -sin_q_elbow;
    (*this)(4,3) = sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    return *this;
}
ForceTransforms::Type_fr_wr1_X_fr_wrist_1::Type_fr_wr1_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_wr1_X_fr_wrist_1& ForceTransforms::Type_fr_wr1_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = -sin_q_wr1;
    (*this)(4,3) = sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    return *this;
}
ForceTransforms::Type_fr_wr2_X_fr_wrist_2::Type_fr_wr2_X_fr_wrist_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_wr2_X_fr_wrist_2& ForceTransforms::Type_fr_wr2_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = cos_q_wr2;
    (*this)(0,1) = -sin_q_wr2;
    (*this)(1,0) = sin_q_wr2;
    (*this)(1,1) = cos_q_wr2;
    (*this)(3,3) = cos_q_wr2;
    (*this)(3,4) = -sin_q_wr2;
    (*this)(4,3) = sin_q_wr2;
    (*this)(4,4) = cos_q_wr2;
    return *this;
}
ForceTransforms::Type_fr_wr3_X_fr_wrist_3::Type_fr_wr3_X_fr_wrist_3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_wr3_X_fr_wrist_3& ForceTransforms::Type_fr_wr3_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = cos_q_wr3;
    (*this)(0,1) = -sin_q_wr3;
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(3,3) = cos_q_wr3;
    (*this)(3,4) = -sin_q_wr3;
    (*this)(4,3) = sin_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_wrist_3::Type_fr_base_X_fr_wrist_3()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_wrist_3& ForceTransforms::Type_fr_base_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = (((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(0,1) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(0,3) = (((((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((- tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(0,4) = ((( tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1)) * sin_q_wr3)+(((((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(0,5) = ((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan)) * cos_q_wr2);
    (*this)(1,0) = (((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(1,1) = (((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(1,3) = ((((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((- tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(1,4) = ((( tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(1,5) = ((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * sin_q_wr2)+(((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan)) * cos_q_wr2);
    (*this)(2,0) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * sin_q_wr3)+(((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(2,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * cos_q_wr3);
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(2,3) = ((((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(2,4) = ((((((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+((((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(2,5) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr2);
    (*this)(3,3) = (((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(3,4) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(3,5) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(4,3) = (((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(4,4) = (((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(4,5) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(5,3) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * sin_q_wr3)+(((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(5,4) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * cos_q_wr3);
    (*this)(5,5) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_shoulder_pan::Type_fr_base_X_fr_shoulder_pan()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_shoulder_pan;    // Maxima DSL: -_k__tz_shoulder_pan
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_shoulder_pan;    // Maxima DSL: _k__tz_shoulder_pan
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_X_fr_shoulder_pan& ForceTransforms::Type_fr_base_X_fr_shoulder_pan::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_shoulder_lift::Type_fr_base_X_fr_shoulder_lift()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_shoulder_lift;    // Maxima DSL: -_k__tx_shoulder_lift
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_shoulder_lift& ForceTransforms::Type_fr_base_X_fr_shoulder_lift::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) =  tx_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,4) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(0,5) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = - tx_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,4) =  tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(1,5) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(3,4) = sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,4) = -cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_elbow::Type_fr_base_X_fr_elbow()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_elbow& ForceTransforms::Type_fr_base_X_fr_elbow::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = sin_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,1) = cos_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) = (( tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_lift * sin_q_shoulder_pan)+( tz_shoulder_pan * sin_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(0,4) = ((- tz_elbow- tx_shoulder_lift) * sin_q_shoulder_lift * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_shoulder_lift)+ tx_elbow) * cos_q_shoulder_pan);
    (*this)(0,5) = ((- tx_elbow * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan;
    (*this)(1,0) = -sin_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = ( tz_shoulder_pan * sin_q_shoulder_lift * sin_q_shoulder_pan)+((- tz_elbow- tx_shoulder_lift) * cos_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(1,4) = ((( tz_shoulder_pan * cos_q_shoulder_lift)+ tx_elbow) * sin_q_shoulder_pan)+(( tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(1,5) = (( tx_elbow * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan;
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    (*this)(2,3) = (- tz_elbow- tx_shoulder_lift) * sin_q_shoulder_lift;
    (*this)(2,4) = (- tz_elbow- tx_shoulder_lift) * cos_q_shoulder_lift;
    (*this)(2,5) =  tx_elbow * sin_q_shoulder_lift;
    (*this)(3,3) = sin_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(3,4) = cos_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,3) = -sin_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(4,4) = -cos_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    (*this)(5,3) = cos_q_shoulder_lift;
    (*this)(5,4) = -sin_q_shoulder_lift;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_wr1::Type_fr_base_X_fr_wr1()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_wr1& ForceTransforms::Type_fr_base_X_fr_wr1::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = ((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,1) = ((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan);
    (*this)(0,4) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan);
    (*this)(0,5) = (( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan;
    (*this)(1,0) = ((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,1) = ((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = ((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(1,4) = (((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(1,5) = ((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan;
    (*this)(2,0) = (cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift);
    (*this)(2,1) = (-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift);
    (*this)(2,3) = ((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(2,4) = (( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift);
    (*this)(2,5) = ((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(3,3) = ((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(3,4) = ((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(3,5) = cos_q_shoulder_pan;
    (*this)(4,3) = ((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(4,4) = ((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(4,5) = sin_q_shoulder_pan;
    (*this)(5,3) = (cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift);
    (*this)(5,4) = (-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_wr2::Type_fr_base_X_fr_wr2()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_wr2& ForceTransforms::Type_fr_base_X_fr_wr2::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,2) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,3) = ((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan);
    (*this)(0,4) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan);
    (*this)(0,5) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,2) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,3) = (((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan);
    (*this)(1,4) = ((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan);
    (*this)(1,5) = (((((- tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)+( tx_elbow * cos_q_elbow)+ tx_wr1) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(2,1) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,2) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,3) = ((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(2,4) = ((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,5) = (((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,5) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,5) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(5,4) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,5) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_wr3::Type_fr_base_X_fr_wr3()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_wr3& ForceTransforms::Type_fr_base_X_fr_wr3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,1) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(0,3) = (- tx_wr3 * cos_q_shoulder_pan * sin_q_wr2)+((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(0,4) = (((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * sin_q_shoulder_pan)) * sin_q_wr2)+((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,5) = ((((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * cos_q_shoulder_pan)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * sin_q_shoulder_pan)) * cos_q_wr2);
    (*this)(1,0) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(1,3) = (- tx_wr3 * sin_q_shoulder_pan * sin_q_wr2)+(((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+(((((- tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)-( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1);
    (*this)(1,4) = ((((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)- tz_shoulder_pan) * cos_q_shoulder_pan)) * sin_q_wr2)+((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * cos_q_wr2)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,5) = ((((((( tz_shoulder_pan * cos_q_elbow * sin_q_shoulder_lift)+( tz_shoulder_pan * sin_q_elbow * cos_q_shoulder_lift)+( tx_elbow * sin_q_elbow)) * sin_q_shoulder_pan)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * sin_q_wr1)+((((( tz_shoulder_pan * sin_q_elbow * sin_q_shoulder_lift)-( tz_shoulder_pan * cos_q_elbow * cos_q_shoulder_lift)-( tx_elbow * cos_q_elbow)- tx_wr1) * sin_q_shoulder_pan)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan)) * cos_q_wr1)-( tx_wr2 * sin_q_shoulder_pan)) * sin_q_wr2)+(((((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(((- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan) * cos_q_shoulder_pan)) * cos_q_wr2);
    (*this)(2,0) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,1) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(2,3) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,4) = ((((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr2)+((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,5) = ((((((- tz_wr1- tz_elbow- tx_shoulder_lift) * cos_q_elbow * sin_q_shoulder_lift)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_elbow * sin_q_shoulder_lift)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr2);
    (*this)(3,3) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,4) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2);
    (*this)(3,5) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(4,3) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,4) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2);
    (*this)(4,5) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(5,3) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,4) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    (*this)(5,5) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    return *this;
}
ForceTransforms::Type_fr_shoulder_X_fr_base::Type_fr_shoulder_X_fr_base()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shoulder_X_fr_base& ForceTransforms::Type_fr_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(0,3) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(0,4) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(1,0) = -sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(1,3) = - tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(1,4) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = sin_q_shoulder_pan;
    (*this)(4,3) = -sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_shoulder::Type_fr_base_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_X_fr_shoulder& ForceTransforms::Type_fr_base_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(0,3) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(0,4) = - tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    (*this)(1,3) =  tz_shoulder_pan * cos_q_shoulder_pan;
    (*this)(1,4) = - tz_shoulder_pan * sin_q_shoulder_pan;
    (*this)(3,3) = cos_q_shoulder_pan;
    (*this)(3,4) = -sin_q_shoulder_pan;
    (*this)(4,3) = sin_q_shoulder_pan;
    (*this)(4,4) = cos_q_shoulder_pan;
    return *this;
}
ForceTransforms::Type_fr_upper_arm_X_fr_shoulder::Type_fr_upper_arm_X_fr_shoulder()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_upper_arm_X_fr_shoulder& ForceTransforms::Type_fr_upper_arm_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(0,2) = cos_q_shoulder_lift;
    (*this)(0,4) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(0,5) = - tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(1,2) = -sin_q_shoulder_lift;
    (*this)(1,4) =  tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(1,5) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(3,4) = -sin_q_shoulder_lift;
    (*this)(3,5) = cos_q_shoulder_lift;
    (*this)(4,4) = -cos_q_shoulder_lift;
    (*this)(4,5) = -sin_q_shoulder_lift;
    return *this;
}
ForceTransforms::Type_fr_shoulder_X_fr_upper_arm::Type_fr_shoulder_X_fr_upper_arm()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_shoulder_X_fr_upper_arm& ForceTransforms::Type_fr_shoulder_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(1,0) = -sin_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(1,3) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(1,4) =  tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    (*this)(2,3) = - tx_shoulder_lift * sin_q_shoulder_lift;
    (*this)(2,4) = - tx_shoulder_lift * cos_q_shoulder_lift;
    (*this)(4,3) = -sin_q_shoulder_lift;
    (*this)(4,4) = -cos_q_shoulder_lift;
    (*this)(5,3) = cos_q_shoulder_lift;
    (*this)(5,4) = -sin_q_shoulder_lift;
    return *this;
}
ForceTransforms::Type_fr_forearm_X_fr_upper_arm::Type_fr_forearm_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_elbow;    // Maxima DSL: -_k__tx_elbow
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_forearm_X_fr_upper_arm& ForceTransforms::Type_fr_forearm_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = sin_q_elbow;
    (*this)(0,3) = - tz_elbow * sin_q_elbow;
    (*this)(0,4) =  tz_elbow * cos_q_elbow;
    (*this)(0,5) =  tx_elbow * sin_q_elbow;
    (*this)(1,0) = -sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(1,3) = - tz_elbow * cos_q_elbow;
    (*this)(1,4) = - tz_elbow * sin_q_elbow;
    (*this)(1,5) =  tx_elbow * cos_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = sin_q_elbow;
    (*this)(4,3) = -sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    return *this;
}
ForceTransforms::Type_fr_upper_arm_X_fr_forearm::Type_fr_upper_arm_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tx_elbow;    // Maxima DSL: -_k__tx_elbow
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_upper_arm_X_fr_forearm& ForceTransforms::Type_fr_upper_arm_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(0,3) = - tz_elbow * sin_q_elbow;
    (*this)(0,4) = - tz_elbow * cos_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(1,3) =  tz_elbow * cos_q_elbow;
    (*this)(1,4) = - tz_elbow * sin_q_elbow;
    (*this)(2,3) =  tx_elbow * sin_q_elbow;
    (*this)(2,4) =  tx_elbow * cos_q_elbow;
    (*this)(3,3) = cos_q_elbow;
    (*this)(3,4) = -sin_q_elbow;
    (*this)(4,3) = sin_q_elbow;
    (*this)(4,4) = cos_q_elbow;
    return *this;
}
ForceTransforms::Type_fr_wrist_1_X_fr_forearm::Type_fr_wrist_1_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_wr1;    // Maxima DSL: -_k__tx_wr1
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_wrist_1_X_fr_forearm& ForceTransforms::Type_fr_wrist_1_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = sin_q_wr1;
    (*this)(0,3) = - tz_wr1 * sin_q_wr1;
    (*this)(0,4) =  tz_wr1 * cos_q_wr1;
    (*this)(0,5) =  tx_wr1 * sin_q_wr1;
    (*this)(1,0) = -sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(1,3) = - tz_wr1 * cos_q_wr1;
    (*this)(1,4) = - tz_wr1 * sin_q_wr1;
    (*this)(1,5) =  tx_wr1 * cos_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = sin_q_wr1;
    (*this)(4,3) = -sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    return *this;
}
ForceTransforms::Type_fr_forearm_X_fr_wrist_1::Type_fr_forearm_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tx_wr1;    // Maxima DSL: -_k__tx_wr1
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_forearm_X_fr_wrist_1& ForceTransforms::Type_fr_forearm_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(0,3) = - tz_wr1 * sin_q_wr1;
    (*this)(0,4) = - tz_wr1 * cos_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(1,3) =  tz_wr1 * cos_q_wr1;
    (*this)(1,4) = - tz_wr1 * sin_q_wr1;
    (*this)(2,3) =  tx_wr1 * sin_q_wr1;
    (*this)(2,4) =  tx_wr1 * cos_q_wr1;
    (*this)(3,3) = cos_q_wr1;
    (*this)(3,4) = -sin_q_wr1;
    (*this)(4,3) = sin_q_wr1;
    (*this)(4,4) = cos_q_wr1;
    return *this;
}
ForceTransforms::Type_fr_wrist_2_X_fr_wrist_1::Type_fr_wrist_2_X_fr_wrist_1()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_2_X_fr_wrist_1& ForceTransforms::Type_fr_wrist_2_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,1) = -sin_q_wr2;
    (*this)(0,2) = cos_q_wr2;
    (*this)(0,4) = - tx_wr2 * cos_q_wr2;
    (*this)(0,5) = - tx_wr2 * sin_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(1,2) = -sin_q_wr2;
    (*this)(1,4) =  tx_wr2 * sin_q_wr2;
    (*this)(1,5) = - tx_wr2 * cos_q_wr2;
    (*this)(3,4) = -sin_q_wr2;
    (*this)(3,5) = cos_q_wr2;
    (*this)(4,4) = -cos_q_wr2;
    (*this)(4,5) = -sin_q_wr2;
    return *this;
}
ForceTransforms::Type_fr_wrist_1_X_fr_wrist_2::Type_fr_wrist_1_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_1_X_fr_wrist_2& ForceTransforms::Type_fr_wrist_1_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(1,0) = -sin_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(1,3) = - tx_wr2 * cos_q_wr2;
    (*this)(1,4) =  tx_wr2 * sin_q_wr2;
    (*this)(2,0) = cos_q_wr2;
    (*this)(2,1) = -sin_q_wr2;
    (*this)(2,3) = - tx_wr2 * sin_q_wr2;
    (*this)(2,4) = - tx_wr2 * cos_q_wr2;
    (*this)(4,3) = -sin_q_wr2;
    (*this)(4,4) = -cos_q_wr2;
    (*this)(5,3) = cos_q_wr2;
    (*this)(5,4) = -sin_q_wr2;
    return *this;
}
ForceTransforms::Type_fr_wrist_3_X_fr_wrist_2::Type_fr_wrist_3_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_3_X_fr_wrist_2& ForceTransforms::Type_fr_wrist_3_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,1) = sin_q_wr3;
    (*this)(0,2) = -cos_q_wr3;
    (*this)(0,4) =  tx_wr3 * cos_q_wr3;
    (*this)(0,5) =  tx_wr3 * sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(1,2) = sin_q_wr3;
    (*this)(1,4) = - tx_wr3 * sin_q_wr3;
    (*this)(1,5) =  tx_wr3 * cos_q_wr3;
    (*this)(3,4) = sin_q_wr3;
    (*this)(3,5) = -cos_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    (*this)(4,5) = sin_q_wr3;
    return *this;
}
ForceTransforms::Type_fr_wrist_2_X_fr_wrist_3::Type_fr_wrist_2_X_fr_wrist_3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_2_X_fr_wrist_3& ForceTransforms::Type_fr_wrist_2_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(1,3) =  tx_wr3 * cos_q_wr3;
    (*this)(1,4) = - tx_wr3 * sin_q_wr3;
    (*this)(2,0) = -cos_q_wr3;
    (*this)(2,1) = sin_q_wr3;
    (*this)(2,3) =  tx_wr3 * sin_q_wr3;
    (*this)(2,4) =  tx_wr3 * cos_q_wr3;
    (*this)(4,3) = sin_q_wr3;
    (*this)(4,4) = cos_q_wr3;
    (*this)(5,3) = -cos_q_wr3;
    (*this)(5,4) = sin_q_wr3;
    return *this;
}

HomogeneousTransforms::Type_fr_shoulder_pan_X_fr_shoulder::Type_fr_shoulder_pan_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_pan_X_fr_shoulder& HomogeneousTransforms::Type_fr_shoulder_pan_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::Type_fr_shoulder_lift_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_lift_X_fr_upper_arm& HomogeneousTransforms::Type_fr_shoulder_lift_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = cos_q_shoulder_lift;
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(1,0) = sin_q_shoulder_lift;
    (*this)(1,1) = cos_q_shoulder_lift;
    return *this;
}
HomogeneousTransforms::Type_fr_elbow_X_fr_forearm::Type_fr_elbow_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_elbow_X_fr_forearm& HomogeneousTransforms::Type_fr_elbow_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    return *this;
}
HomogeneousTransforms::Type_fr_wr1_X_fr_wrist_1::Type_fr_wr1_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wr1_X_fr_wrist_1& HomogeneousTransforms::Type_fr_wr1_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    return *this;
}
HomogeneousTransforms::Type_fr_wr2_X_fr_wrist_2::Type_fr_wr2_X_fr_wrist_2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wr2_X_fr_wrist_2& HomogeneousTransforms::Type_fr_wr2_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = cos_q_wr2;
    (*this)(0,1) = -sin_q_wr2;
    (*this)(1,0) = sin_q_wr2;
    (*this)(1,1) = cos_q_wr2;
    return *this;
}
HomogeneousTransforms::Type_fr_wr3_X_fr_wrist_3::Type_fr_wr3_X_fr_wrist_3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wr3_X_fr_wrist_3& HomogeneousTransforms::Type_fr_wr3_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = cos_q_wr3;
    (*this)(0,1) = -sin_q_wr3;
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_wrist_3::Type_fr_base_X_fr_wrist_3()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_wrist_3& HomogeneousTransforms::Type_fr_base_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,0) = (((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(0,1) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(0,3) = ((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+( tx_wr3 * cos_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(1,0) = (((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr3);
    (*this)(1,1) = (((((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr3)+(((((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2)) * cos_q_wr3);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(1,3) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+( tx_wr3 * sin_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_pan)+(((((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(2,0) = (((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * sin_q_wr3)+(((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr3);
    (*this)(2,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr3)+(((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2 * cos_q_wr3);
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(2,3) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)-( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_shoulder_pan::Type_fr_base_X_fr_shoulder_pan()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_shoulder_pan;    // Maxima DSL: _k__tz_shoulder_pan
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_shoulder_pan& HomogeneousTransforms::Type_fr_base_X_fr_shoulder_pan::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_shoulder_lift::Type_fr_base_X_fr_shoulder_lift()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_shoulder_pan;    // Maxima DSL: _k__tz_shoulder_pan
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_shoulder_lift& HomogeneousTransforms::Type_fr_base_X_fr_shoulder_lift::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) =  tx_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) =  tx_shoulder_lift * sin_q_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_elbow::Type_fr_base_X_fr_elbow()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_elbow& HomogeneousTransforms::Type_fr_base_X_fr_elbow::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,0) = sin_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,1) = cos_q_shoulder_lift * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) = ( tx_elbow * sin_q_shoulder_lift * sin_q_shoulder_pan)+(( tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(1,0) = -sin_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,1) = -cos_q_shoulder_lift * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = (( tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_pan)-( tx_elbow * sin_q_shoulder_lift * cos_q_shoulder_pan);
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    (*this)(2,3) = ( tx_elbow * cos_q_shoulder_lift)+ tz_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_wr1::Type_fr_base_X_fr_wr1()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_wr1& HomogeneousTransforms::Type_fr_base_X_fr_wr1::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = ((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,1) = ((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) = ((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(1,0) = ((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,1) = ((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = (( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_pan)+(((((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(2,0) = (cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift);
    (*this)(2,1) = (-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift);
    (*this)(2,3) = (- tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_wr2::Type_fr_base_X_fr_wr2()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_wr2& HomogeneousTransforms::Type_fr_base_X_fr_wr2::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,2) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,3) = ((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,2) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,3) = ((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_pan)+(((((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(2,1) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,2) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,3) = (((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)-( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_wr3::Type_fr_base_X_fr_wr3()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_wr3& HomogeneousTransforms::Type_fr_base_X_fr_wr3::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,0) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,1) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(cos_q_shoulder_pan * sin_q_wr2);
    (*this)(0,2) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(0,3) = ((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+( tx_wr3 * cos_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(1,0) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,1) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-(sin_q_shoulder_pan * sin_q_wr2);
    (*this)(1,2) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(1,3) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+( tx_wr3 * sin_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * sin_q_shoulder_pan)+(((((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(2,0) = (((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,1) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    (*this)(2,2) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(2,3) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)-( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)+ tz_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_shoulder_X_fr_base::Type_fr_shoulder_X_fr_base()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_shoulder_pan;    // Maxima DSL: -_k__tz_shoulder_pan
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_X_fr_base& HomogeneousTransforms::Type_fr_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = sin_q_shoulder_pan;
    (*this)(1,0) = -sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_shoulder::Type_fr_base_X_fr_shoulder()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_shoulder_pan;    // Maxima DSL: _k__tz_shoulder_pan
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_shoulder& HomogeneousTransforms::Type_fr_base_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan  = ScalarTraits::sin( q(SHOULDER_PAN) );
    Scalar cos_q_shoulder_pan  = ScalarTraits::cos( q(SHOULDER_PAN) );
    (*this)(0,0) = cos_q_shoulder_pan;
    (*this)(0,1) = -sin_q_shoulder_pan;
    (*this)(1,0) = sin_q_shoulder_pan;
    (*this)(1,1) = cos_q_shoulder_pan;
    return *this;
}
HomogeneousTransforms::Type_fr_upper_arm_X_fr_shoulder::Type_fr_upper_arm_X_fr_shoulder()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_shoulder_lift;    // Maxima DSL: -_k__tx_shoulder_lift
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_upper_arm_X_fr_shoulder& HomogeneousTransforms::Type_fr_upper_arm_X_fr_shoulder::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(0,1) = -sin_q_shoulder_lift;
    (*this)(0,2) = cos_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(1,2) = -sin_q_shoulder_lift;
    return *this;
}
HomogeneousTransforms::Type_fr_shoulder_X_fr_upper_arm::Type_fr_shoulder_X_fr_upper_arm()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_shoulder_lift;    // Maxima DSL: _k__tx_shoulder_lift
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_X_fr_upper_arm& HomogeneousTransforms::Type_fr_shoulder_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift  = ScalarTraits::sin( q(SHOULDER_LIFT) );
    Scalar cos_q_shoulder_lift  = ScalarTraits::cos( q(SHOULDER_LIFT) );
    (*this)(1,0) = -sin_q_shoulder_lift;
    (*this)(1,1) = -cos_q_shoulder_lift;
    (*this)(2,0) = cos_q_shoulder_lift;
    (*this)(2,1) = -sin_q_shoulder_lift;
    return *this;
}
HomogeneousTransforms::Type_fr_forearm_X_fr_upper_arm::Type_fr_forearm_X_fr_upper_arm()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_elbow;    // Maxima DSL: -_k__tz_elbow
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_forearm_X_fr_upper_arm& HomogeneousTransforms::Type_fr_forearm_X_fr_upper_arm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = sin_q_elbow;
    (*this)(0,3) = - tx_elbow * cos_q_elbow;
    (*this)(1,0) = -sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    (*this)(1,3) =  tx_elbow * sin_q_elbow;
    return *this;
}
HomogeneousTransforms::Type_fr_upper_arm_X_fr_forearm::Type_fr_upper_arm_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow;    // Maxima DSL: _k__tx_elbow
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_elbow;    // Maxima DSL: _k__tz_elbow
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_upper_arm_X_fr_forearm& HomogeneousTransforms::Type_fr_upper_arm_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_elbow  = ScalarTraits::sin( q(ELBOW) );
    Scalar cos_q_elbow  = ScalarTraits::cos( q(ELBOW) );
    (*this)(0,0) = cos_q_elbow;
    (*this)(0,1) = -sin_q_elbow;
    (*this)(1,0) = sin_q_elbow;
    (*this)(1,1) = cos_q_elbow;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_1_X_fr_forearm::Type_fr_wrist_1_X_fr_forearm()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_wr1;    // Maxima DSL: -_k__tz_wr1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_1_X_fr_forearm& HomogeneousTransforms::Type_fr_wrist_1_X_fr_forearm::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = sin_q_wr1;
    (*this)(0,3) = - tx_wr1 * cos_q_wr1;
    (*this)(1,0) = -sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    (*this)(1,3) =  tx_wr1 * sin_q_wr1;
    return *this;
}
HomogeneousTransforms::Type_fr_forearm_X_fr_wrist_1::Type_fr_forearm_X_fr_wrist_1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_wr1;    // Maxima DSL: _k__tx_wr1
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_wr1;    // Maxima DSL: _k__tz_wr1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_forearm_X_fr_wrist_1& HomogeneousTransforms::Type_fr_forearm_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr1  = ScalarTraits::sin( q(WR1) );
    Scalar cos_q_wr1  = ScalarTraits::cos( q(WR1) );
    (*this)(0,0) = cos_q_wr1;
    (*this)(0,1) = -sin_q_wr1;
    (*this)(1,0) = sin_q_wr1;
    (*this)(1,1) = cos_q_wr1;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_1::Type_fr_wrist_2_X_fr_wrist_1()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_wr2;    // Maxima DSL: -_k__tx_wr2
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_1& HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_1::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(0,1) = -sin_q_wr2;
    (*this)(0,2) = cos_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(1,2) = -sin_q_wr2;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_1_X_fr_wrist_2::Type_fr_wrist_1_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_wr2;    // Maxima DSL: _k__tx_wr2
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_1_X_fr_wrist_2& HomogeneousTransforms::Type_fr_wrist_1_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr2  = ScalarTraits::sin( q(WR2) );
    Scalar cos_q_wr2  = ScalarTraits::cos( q(WR2) );
    (*this)(1,0) = -sin_q_wr2;
    (*this)(1,1) = -cos_q_wr2;
    (*this)(2,0) = cos_q_wr2;
    (*this)(2,1) = -sin_q_wr2;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_3_X_fr_wrist_2::Type_fr_wrist_3_X_fr_wrist_2()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_wr3;    // Maxima DSL: -_k__tx_wr3
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_3_X_fr_wrist_2& HomogeneousTransforms::Type_fr_wrist_3_X_fr_wrist_2::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(0,1) = sin_q_wr3;
    (*this)(0,2) = -cos_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(1,2) = sin_q_wr3;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_3::Type_fr_wrist_2_X_fr_wrist_3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_wr3;    // Maxima DSL: _k__tx_wr3
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_3& HomogeneousTransforms::Type_fr_wrist_2_X_fr_wrist_3::update(const state_t& q)
{
    Scalar sin_q_wr3  = ScalarTraits::sin( q(WR3) );
    Scalar cos_q_wr3  = ScalarTraits::cos( q(WR3) );
    (*this)(1,0) = sin_q_wr3;
    (*this)(1,1) = cos_q_wr3;
    (*this)(2,0) = -cos_q_wr3;
    (*this)(2,1) = sin_q_wr3;
    return *this;
}

