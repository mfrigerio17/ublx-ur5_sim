#ifndef RCG_UR5_DECLARATIONS_H_
#define RCG_UR5_DECLARATIONS_H_

#include "rbd_types.h"

namespace ur5 {
namespace rcg {

static constexpr int JointSpaceDimension = 6;
static constexpr int jointsCount = 6;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 7;

typedef Matrix<6, 1> Column6d;
typedef Column6d JointState;

enum JointIdentifiers {
    SHOULDER_PAN = 0
    , SHOULDER_LIFT
    , ELBOW
    , WR1
    , WR2
    , WR3
};

enum LinkIdentifiers {
    BASE = 0
    , SHOULDER
    , UPPER_ARM
    , FOREARM
    , WRIST_1
    , WRIST_2
    , WRIST_3
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {SHOULDER_PAN,SHOULDER_LIFT,ELBOW,WR1,WR2,WR3};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,SHOULDER,UPPER_ARM,FOREARM,WRIST_1,WRIST_2,WRIST_3};

}
}
#endif
