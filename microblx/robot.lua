return block 
{
    name = "robot_interface",
    license = "BSD 3-Clause",
    meta_data = "a generic, empty dummy robot interface block",

    port_cache = true,

    configurations = {
	    {
    	    name = "joint_state_size",
	        type_name = "int", min=1,
	        doc = "number of degrees of freedom"
	    },
	    {
    	    name = "misc_sensors_count",
	        type_name = "int", min=0,
	        doc = "number of miscellaneous sensed quantities on the robot"
	    },
      },

    ports = {
        { name = "q"   , out_type_name = "double", doc="joint status vector" },
        { name = "qd"  , out_type_name = "double", doc="joint velocity vector" },
        { name = "load", out_type_name = "double", doc="measured joint effort" },
        { name = "misc_sensors", out_type_name = "double", doc="miscellaneous sensor inputs" },

        { name = "tau", in_type_name = "double", doc="joint force command" },

        -- this one should not be here, I think, but for now it is simpler
        { name = "time", out_type_name = "double", doc="physical time flow" },
    },

    operations = { start=true, stop=true, step=true }
}
