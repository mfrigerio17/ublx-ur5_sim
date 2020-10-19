#include "graphics_bridge.hpp"

#include <iostream>
#include <ur5/rcg/declarations.h>
#include <ur5/rcg/transforms.h>

#include <zmq.hpp>
#include <meshcatcpp/zmqclient.h>
#include <meshcatcpp/commands.h>

zmq::context_t context(1);


/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct graphics_bridge_info
{
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct graphics_bridge_port_cache ports;
	
	graphics_bridge_info() : zmqclient(context) {
	    q.setZero();
	}

	meshcat::ZMQClient zmqclient;
	meshcat::PckSetTransform command_packet;

    int joint_state_size = 0;
    ur5::rcg::HomogeneousTransforms xh;
	ur5::rcg::JointState q;
};

inline graphics_bridge_info& private_data(ubx_block_t *b)
{
	return *static_cast<graphics_bridge_info*>(b->private_data);
}

static std::string meshcat_path_base = u8"/meshcat/base";
static std::string meshcat_path_shoulder = meshcat_path_base + "/shoulder_pan/shoulder";
static std::string meshcat_path_upperarm = meshcat_path_shoulder + "/shoulder_lift/upper_arm";
static std::string meshcat_path_forearm  = meshcat_path_upperarm + "/elbow/forearm";
static std::string meshcat_path_wrist1   = meshcat_path_forearm + "/wr1/wrist_1";
static std::string meshcat_path_wrist2   = meshcat_path_wrist1 + "/wr2/wrist_2";
static std::string meshcat_path_wrist3   = meshcat_path_wrist2 + "/wr3/wrist_3";



/* init */
int graphics_bridge_init(ubx_block_t *b)
{
	int ret = -1;
    long len = 0;
	struct graphics_bridge_info *inf = new graphics_bridge_info();
	if ( inf==NULL ) {
		ubx_err(b, "graphics_bridge: failed to alloc memory");
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
	    ubx_inport_resize(inf->ports.q, inf->joint_state_size)
	    ) {
		goto out;
	}


	ret=0;
out:
	return ret;
}


int graphics_bridge_start(ubx_block_t *b)
{
    int ret = 0;
    graphics_bridge_info& data = private_data(b);
    try {
        data.zmqclient.connect ("tcp://127.0.0.1:6000");
    }
    catch(zmq::error_t& exc) {
        std::cerr << "Could not connect to ZMQ socket" << std::endl;
        std::cerr << exc.what() << std::endl;
        ret = -1;
    }

	return ret;
}

/* stop */
void graphics_bridge_stop(ubx_block_t *b)
{
    /* struct graphics_bridge_info *inf = (struct graphics_bridge_info*) b->private_data; */
    //ubx_info(b, "%s", __func__);
}

/* cleanup */
void graphics_bridge_cleanup(ubx_block_t *b)
{
	delete( static_cast<graphics_bridge_info*>(b->private_data) );
}



void graphics_bridge_step(ubx_block_t *b)
{
    graphics_bridge_info& data = private_data(b);
    read_double_array(data.ports.q, data.q.data(), data.joint_state_size);
    //std::cout << data.q.transpose() << std::endl;
    //data.q(0) += 0.2;
    data.xh.fr_shoulder_pan_X_fr_shoulder(data.q);
    data.xh.fr_shoulder_lift_X_fr_upper_arm(data.q);
    data.xh.fr_elbow_X_fr_forearm(data.q);
    data.xh.fr_wr1_X_fr_wrist_1(data.q);
    data.xh.fr_wr2_X_fr_wrist_2(data.q);
    data.xh.fr_wr3_X_fr_wrist_3(data.q);
    //std::cout << data.xh.fr_shoulder_pan_X_fr_shoulder << std::endl << std::endl;
    //data.xh.fr_shoulder_pan_X_fr_shoulder(0,3) += 0.2;
    const meshcat::PackedDataBuffer& buff = data.command_packet.pack_payload(meshcat_path_shoulder, data.xh.fr_shoulder_pan_X_fr_shoulder.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_shoulder, buff);

    data.command_packet.pack_payload(meshcat_path_upperarm, data.xh.fr_shoulder_lift_X_fr_upper_arm.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_upperarm, buff);

    data.command_packet.pack_payload(meshcat_path_forearm, data.xh.fr_elbow_X_fr_forearm.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_forearm, buff);

    data.command_packet.pack_payload(meshcat_path_wrist1, data.xh.fr_wr1_X_fr_wrist_1.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_wrist1, buff);

    data.command_packet.pack_payload(meshcat_path_wrist2, data.xh.fr_wr2_X_fr_wrist_2.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_wrist2, buff);

    data.command_packet.pack_payload(meshcat_path_wrist3, data.xh.fr_wr3_X_fr_wrist_3.data());
    data.zmqclient.send_command(meshcat::cmd_set_transform, meshcat_path_wrist3, buff);
}

