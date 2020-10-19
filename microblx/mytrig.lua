return block
{
    name="ext_time_trig",
    license="BSD-3-Clause",
    meta_data="External-time-signal-based trigger",
    port_cache=true,

    types = { },

    configurations= {
    },

    ports = {
	    { name="time", in_type_name="double", in_data_len=1, doc="time flow signal" },
    },

    operations = { start=true, stop=true, step=true }
}
