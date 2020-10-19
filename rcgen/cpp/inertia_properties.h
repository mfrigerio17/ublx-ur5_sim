#ifndef RCG_UR5_INERTIA_PROPERTIES_H_
#define RCG_UR5_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace ur5 {
namespace rcg {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_shoulder() const;
        const InertiaMatrix& getTensor_upper_arm() const;
        const InertiaMatrix& getTensor_forearm() const;
        const InertiaMatrix& getTensor_wrist_1() const;
        const InertiaMatrix& getTensor_wrist_2() const;
        const InertiaMatrix& getTensor_wrist_3() const;
        Scalar getMass_shoulder() const;
        Scalar getMass_upper_arm() const;
        Scalar getMass_forearm() const;
        Scalar getMass_wrist_1() const;
        Scalar getMass_wrist_2() const;
        Scalar getMass_wrist_3() const;
        const Vector3& getCOM_shoulder() const;
        const Vector3& getCOM_upper_arm() const;
        const Vector3& getCOM_forearm() const;
        const Vector3& getCOM_wrist_1() const;
        const Vector3& getCOM_wrist_2() const;
        const Vector3& getCOM_wrist_3() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot ur5,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_shoulder;
        InertiaMatrix tensor_upper_arm;
        InertiaMatrix tensor_forearm;
        InertiaMatrix tensor_wrist_1;
        InertiaMatrix tensor_wrist_2;
        InertiaMatrix tensor_wrist_3;
        Vector3 com_shoulder;
        Vector3 com_upper_arm;
        Vector3 com_forearm;
        Vector3 com_wrist_1;
        Vector3 com_wrist_2;
        Vector3 com_wrist_3;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_shoulder() const {
    return this->tensor_shoulder;
}
inline const InertiaMatrix& InertiaProperties::getTensor_upper_arm() const {
    return this->tensor_upper_arm;
}
inline const InertiaMatrix& InertiaProperties::getTensor_forearm() const {
    return this->tensor_forearm;
}
inline const InertiaMatrix& InertiaProperties::getTensor_wrist_1() const {
    return this->tensor_wrist_1;
}
inline const InertiaMatrix& InertiaProperties::getTensor_wrist_2() const {
    return this->tensor_wrist_2;
}
inline const InertiaMatrix& InertiaProperties::getTensor_wrist_3() const {
    return this->tensor_wrist_3;
}
inline Scalar InertiaProperties::getMass_shoulder() const {
    return this->tensor_shoulder.getMass();
}
inline Scalar InertiaProperties::getMass_upper_arm() const {
    return this->tensor_upper_arm.getMass();
}
inline Scalar InertiaProperties::getMass_forearm() const {
    return this->tensor_forearm.getMass();
}
inline Scalar InertiaProperties::getMass_wrist_1() const {
    return this->tensor_wrist_1.getMass();
}
inline Scalar InertiaProperties::getMass_wrist_2() const {
    return this->tensor_wrist_2.getMass();
}
inline Scalar InertiaProperties::getMass_wrist_3() const {
    return this->tensor_wrist_3.getMass();
}
inline const Vector3& InertiaProperties::getCOM_shoulder() const {
    return this->com_shoulder;
}
inline const Vector3& InertiaProperties::getCOM_upper_arm() const {
    return this->com_upper_arm;
}
inline const Vector3& InertiaProperties::getCOM_forearm() const {
    return this->com_forearm;
}
inline const Vector3& InertiaProperties::getCOM_wrist_1() const {
    return this->com_wrist_1;
}
inline const Vector3& InertiaProperties::getCOM_wrist_2() const {
    return this->com_wrist_2;
}
inline const Vector3& InertiaProperties::getCOM_wrist_3() const {
    return this->com_wrist_3;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_shoulder + m_upper_arm + m_forearm + m_wrist_1 + m_wrist_2 + m_wrist_3;
}

}
}

#endif
