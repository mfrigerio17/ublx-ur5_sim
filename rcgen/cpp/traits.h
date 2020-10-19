#ifndef RCG__UR5_TRAITS_H_
#define RCG__UR5_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace ur5 {
namespace rcg {
struct Traits {
    typedef typename ur5::rcg::ScalarTraits ScalarTraits;

    typedef typename ur5::rcg::JointState JointState;

    typedef typename ur5::rcg::JointIdentifiers JointID;
    typedef typename ur5::rcg::LinkIdentifiers  LinkID;

    typedef typename ur5::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename ur5::rcg::MotionTransforms MotionTransforms;
    typedef typename ur5::rcg::ForceTransforms ForceTransforms;

    typedef typename ur5::rcg::InertiaProperties InertiaProperties;
    typedef typename ur5::rcg::ForwardDynamics FwdDynEngine;
    typedef typename ur5::rcg::InverseDynamics InvDynEngine;
    typedef typename ur5::rcg::JSIM JSIM;

    static const int joints_count = ur5::rcg::jointsCount;
    static const int links_count  = ur5::rcg::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return ur5::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return ur5::rcg::orderedLinkIDs;
}

}
}

#endif
