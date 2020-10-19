#ifndef RCG_UR5_LINK_DATA_MAP_H_
#define RCG_UR5_LINK_DATA_MAP_H_

#include "declarations.h"

namespace ur5 {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE] = rhs[BASE];
    data[SHOULDER] = rhs[SHOULDER];
    data[UPPER_ARM] = rhs[UPPER_ARM];
    data[FOREARM] = rhs[FOREARM];
    data[WRIST_1] = rhs[WRIST_1];
    data[WRIST_2] = rhs[WRIST_2];
    data[WRIST_3] = rhs[WRIST_3];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE] = value;
    data[SHOULDER] = value;
    data[UPPER_ARM] = value;
    data[FOREARM] = value;
    data[WRIST_1] = value;
    data[WRIST_2] = value;
    data[WRIST_3] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base = "
    << map[BASE]
    << "   shoulder = "
    << map[SHOULDER]
    << "   upper_arm = "
    << map[UPPER_ARM]
    << "   forearm = "
    << map[FOREARM]
    << "   wrist_1 = "
    << map[WRIST_1]
    << "   wrist_2 = "
    << map[WRIST_2]
    << "   wrist_3 = "
    << map[WRIST_3]
    ;
    return out;
}

}
}
#endif
