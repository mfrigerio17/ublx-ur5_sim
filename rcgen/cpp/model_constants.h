#ifndef RCG_UR5_MODEL_CONSTANTS_H_
#define RCG_UR5_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace ur5 {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tz_shoulder_pan = 0.08915899693965912;
const Scalar tx_shoulder_lift = 0.1358499974012375;
const Scalar tx_elbow = 0.42500001192092896;
const Scalar tz_elbow = -0.11969999969005585;
const Scalar tx_wr1 = 0.3922500014305115;
const Scalar tz_wr1 = 0.09314999729394913;
const Scalar tx_wr2 = 0.09475000202655792;
const Scalar tx_wr3 = 0.08250000327825546;
const Scalar m_base = 4.0;
const Scalar comz_base = 0.02500000037252903;
const Scalar ix_base = 0.006930000148713589;
const Scalar iy_base = 0.006930000148713589;
const Scalar iz_base = 0.007199999876320362;
const Scalar m_shoulder = 3.700000047683716;
const Scalar comy_shoulder = 0.001930000027641654;
const Scalar comz_shoulder = -0.025609999895095825;
const Scalar ix_shoulder = 0.01271000038832426;
const Scalar iy_shoulder = 0.012690000236034393;
const Scalar iyz_shoulder = -1.8000000272877514E-4;
const Scalar iz_shoulder = 0.006670000031590462;
const Scalar m_upper_arm = 8.392999649047852;
const Scalar comy_upper_arm = -0.02419999986886978;
const Scalar comz_upper_arm = 0.21250000596046448;
const Scalar ix_upper_arm = 0.6093699932098389;
const Scalar iy_upper_arm = 0.604449987411499;
const Scalar iyz_upper_arm = -0.04315999895334244;
const Scalar iz_upper_arm = 0.017149999737739563;
const Scalar m_forearm = 2.2750000953674316;
const Scalar comy_forearm = 0.026499999687075615;
const Scalar comz_forearm = 0.11992999911308289;
const Scalar ix_forearm = 0.08263000100851059;
const Scalar iy_forearm = 0.0810299962759018;
const Scalar iyz_forearm = 0.007230000104755163;
const Scalar iz_forearm = 0.0034199999645352364;
const Scalar m_wrist_1 = 1.218999981880188;
const Scalar comy_wrist_1 = 0.11095000058412552;
const Scalar comz_wrist_1 = 0.016340000554919243;
const Scalar ix_wrist_1 = 0.017410000786185265;
const Scalar iy_wrist_1 = 0.002409999957308173;
const Scalar iyz_wrist_1 = 0.0022100000642240047;
const Scalar iz_wrist_1 = 0.01624000072479248;
const Scalar m_wrist_2 = 1.218999981880188;
const Scalar comy_wrist_2 = 0.0017999999690800905;
const Scalar comz_wrist_2 = 0.1109900027513504;
const Scalar ix_wrist_2 = 0.017100000753998756;
const Scalar iy_wrist_2 = 0.017100000753998756;
const Scalar iyz_wrist_2 = 2.3999999393709004E-4;
const Scalar iz_wrist_2 = 0.0012400000123307109;
const Scalar m_wrist_3 = 0.18790000677108765;
const Scalar comy_wrist_3 = 0.0011599999852478504;
const Scalar ix_wrist_3 = 3.1999999191612005E-4;
const Scalar iy_wrist_3 = 3.1999999191612005E-4;
const Scalar iz_wrist_3 = 1.900000061141327E-4;

}
}
#endif
