#ifndef UR5_JACOBIANS_H_
#define UR5_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace ur5 {
namespace rcg {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        
        struct Type_fr_base_J_fr_wrist_3 : public JacobianT<6, Type_fr_base_J_fr_wrist_3>
        {
            Type_fr_base_J_fr_wrist_3();
            const Type_fr_base_J_fr_wrist_3& update(const JointState&);
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_base_J_fr_wrist_3 fr_base_J_fr_wrist_3;

    protected:
        Parameters params;

};


}
}

#endif
