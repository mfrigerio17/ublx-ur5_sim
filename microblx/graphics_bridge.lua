return block 
{
    name = "graphics_bridge",
    license = "BSD 3-Clause",
    meta_data = "",

    port_cache = true,

    configurations = {
	    {
    	    name = "joint_state_size",
	        type_name = "int", min=1,
	        doc = "number of degrees of freedom"
	    },
      },

    ports = {
        { name = "q"   , in_type_name = "double", doc="joint status vector" },
    },

    operations = { start=true, stop=true, step=true }
}
