#include "robot_interface.hpp"

#include <iostream>
#include <cmath>
#include <ur5/rcg/declarations.h>
#include <ur5/rcg/forward_dynamics.h>

#include <boost/numeric/odeint/algebra/algebra_dispatcher.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/integrate/integrate_n_steps.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using namespace ur5;
using namespace ur5::rcg;
namespace odeint = boost::numeric::odeint;



using state = Eigen::Matrix<double, 12, 1>;
#define X  block<6,1>(0,0)
#define XD block<6,1>(6,0)

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct robot_interface_info
{
	InertiaProperties ip;
	MotionTransforms xm;
 	ForwardDynamics fd;
	
	robot_interface_info() : ip(), xm(), fd(ip,xm) {
	    reset();
	}

	void reset() {
	    st0.setZero();
	    tau.setZero();
	    time = 0;
	}
	
	int joint_state_size = 0;
	state st0;
	JointState tau;
    
    double time = 0;

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct robot_interface_port_cache ports;
	
	/* The rhs of x' = f(x) */
    void dyn( const state& st, state& dst_dt, const double time )
    {
        JointState qdd;
        fd.fd(qdd, st.X, st.XD, tau);
        dst_dt.X  = st.XD;
        dst_dt.XD = qdd;
    }
};

inline robot_interface_info& private_data(ubx_block_t *b)
{
	return *static_cast<robot_interface_info*>(b->private_data);
}


/* init */
int robot_interface_init(ubx_block_t *b)
{
	int ret = -1;
	long len = 0;
	robot_interface_info *inf;

	/* allocate memory for the block local state */
	if ((inf = new robot_interface_info())  == NULL)
	{
		ubx_err(b, "robot_interface: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	update_port_cache(b, &inf->ports);
	

	const int* joint_state_size;
	len = cfg_getptr_int(b, "joint_state_size", &joint_state_size);
	if (len < 0) goto out;

	inf->joint_state_size = (len > 0) ? *joint_state_size : 1;

	/* resize ports */
	if (
	    ubx_outport_resize(inf->ports.qd, inf->joint_state_size) ||
	    ubx_outport_resize(inf->ports.q , inf->joint_state_size) ||
	    ubx_inport_resize (inf->ports.tau,inf->joint_state_size) )
	{
		goto out;
	}

	ret=0;
out:
	return ret;
}


int robot_interface_start(ubx_block_t *b)
{
	int ret = 0;
	return ret;
}



void robot_interface_stop(ubx_block_t *b)
{
    robot_interface_info& data = private_data(b);
    data.reset();
}



void robot_interface_cleanup(ubx_block_t *b)
{
	delete(static_cast<robot_interface_info*>(b->private_data));
}

void normalize_angle(double& angle)
{
    angle = atan2(sin(angle), cos(angle));
}

namespace pl = std::placeholders;
void robot_interface_step(ubx_block_t *b)
{
	robot_interface_info& data = private_data(b);

	auto ptr_dyn = std::bind( &robot_interface_info::dyn , data , pl::_1 , pl::_2 , pl::_3 );
    double dt = 0.001;

    read_double_array(data.ports.tau, data.tau.data(), data.joint_state_size);
    odeint::runge_kutta4<state> stepper;
    /*size_t steps = */odeint::integrate_n_steps(stepper, ptr_dyn, data.st0, data.time, /*data.time+dt,*/ dt/10, 10);
    //std::cout << steps << std::endl;
    data.time += dt;
    normalize_angle(data.st0.X(0));
    normalize_angle(data.st0.X(1));
    normalize_angle(data.st0.X(2));
    normalize_angle(data.st0.X(3));
    normalize_angle(data.st0.X(4));
    normalize_angle(data.st0.X(5));
    write_double_array(data.ports.q , data.st0.X.data() , data.joint_state_size);
    write_double_array(data.ports.qd, data.st0.XD.data(), data.joint_state_size);

    write_double(data.ports.time, &(data.time));
}

