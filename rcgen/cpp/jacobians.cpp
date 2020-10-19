#include "jacobians.h"

ur5::rcg::Jacobians::Jacobians()
:    fr_base_J_fr_wrist_3()
{}

void ur5::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

ur5::rcg::Jacobians::Type_fr_base_J_fr_wrist_3::Type_fr_base_J_fr_wrist_3()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,5) = 0.0;
}

const ur5::rcg::Jacobians::Type_fr_base_J_fr_wrist_3& ur5::rcg::Jacobians::Type_fr_base_J_fr_wrist_3::update(const JointState& q)
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
    (*this)(0,1) = cos_q_shoulder_pan;
    (*this)(0,2) = cos_q_shoulder_pan;
    (*this)(0,3) = cos_q_shoulder_pan;
    (*this)(0,4) = (((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(0,5) = (((((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+(((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(cos_q_shoulder_pan * cos_q_wr2);
    (*this)(1,1) = sin_q_shoulder_pan;
    (*this)(1,2) = sin_q_shoulder_pan;
    (*this)(1,3) = sin_q_shoulder_pan;
    (*this)(1,4) = (((sin_q_elbow * sin_q_shoulder_lift)-(cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(1,5) = (((((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(sin_q_shoulder_pan * cos_q_wr2);
    (*this)(2,4) = (((-cos_q_elbow * sin_q_shoulder_lift)-(sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1);
    (*this)(2,5) = ((((cos_q_elbow * cos_q_shoulder_lift)-(sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+(((cos_q_elbow * sin_q_shoulder_lift)+(sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2;
    (*this)(3,0) = ((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)-( tx_wr3 * sin_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((- tz_wr1- tz_elbow- tx_shoulder_lift) * sin_q_shoulder_pan)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(3,1) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * cos_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan);
    (*this)(3,2) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan);
    (*this)(3,3) = ((((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1);
    (*this)(3,4) = ((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-( tx_wr3 * cos_q_shoulder_pan * sin_q_wr2);
    (*this)(4,0) = ((((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+( tx_wr3 * cos_q_shoulder_pan * cos_q_wr2)+((( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan * cos_q_wr1)+((((( tx_wr1 * cos_q_elbow)+ tx_elbow) * sin_q_shoulder_lift)+( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_shoulder_pan)+(( tz_wr1+ tz_elbow+ tx_shoulder_lift) * cos_q_shoulder_pan);
    (*this)(4,1) = ((((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(4,2) = ((((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)+((( tx_wr1 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr1 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan);
    (*this)(4,3) = ((((( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+(((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1);
    (*this)(4,4) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_shoulder_pan * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_shoulder_pan * cos_q_wr1)) * cos_q_wr2)-( tx_wr3 * sin_q_shoulder_pan * sin_q_wr2);
    (*this)(5,1) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)+(((- tx_wr1 * cos_q_elbow)- tx_elbow) * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(5,2) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)-( tx_wr1 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr1 * sin_q_elbow * cos_q_shoulder_lift);
    (*this)(5,3) = (((((- tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * cos_q_wr1)) * sin_q_wr2)+((( tx_wr2 * sin_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * cos_q_elbow * cos_q_shoulder_lift)) * sin_q_wr1)+(((- tx_wr2 * cos_q_elbow * sin_q_shoulder_lift)-( tx_wr2 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1);
    (*this)(5,4) = (((( tx_wr3 * cos_q_elbow * cos_q_shoulder_lift)-( tx_wr3 * sin_q_elbow * sin_q_shoulder_lift)) * sin_q_wr1)+((( tx_wr3 * cos_q_elbow * sin_q_shoulder_lift)+( tx_wr3 * sin_q_elbow * cos_q_shoulder_lift)) * cos_q_wr1)) * cos_q_wr2;
    return *this;
}

