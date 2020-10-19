#ifndef RCG_UR5_JOINT_DATA_MAP_H_
#define RCG_UR5_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace ur5 {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[SHOULDER_PAN] = rhs[SHOULDER_PAN];
    data[SHOULDER_LIFT] = rhs[SHOULDER_LIFT];
    data[ELBOW] = rhs[ELBOW];
    data[WR1] = rhs[WR1];
    data[WR2] = rhs[WR2];
    data[WR3] = rhs[WR3];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[SHOULDER_PAN] = value;
    data[SHOULDER_LIFT] = value;
    data[ELBOW] = value;
    data[WR1] = value;
    data[WR2] = value;
    data[WR3] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   shoulder_pan = "
    << map[SHOULDER_PAN]
    << "   shoulder_lift = "
    << map[SHOULDER_LIFT]
    << "   elbow = "
    << map[ELBOW]
    << "   wr1 = "
    << map[WR1]
    << "   wr2 = "
    << map[WR2]
    << "   wr3 = "
    << map[WR3]
    ;
    return out;
}

}
}
#endif
