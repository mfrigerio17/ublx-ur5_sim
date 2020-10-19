return block
{
      name="signal_generator",
      license="BSD-3-Clause",
      meta_data="",
      port_cache=true,

      configurations= {
	 { name="data_len", type_name="unsigned int", doc="length of signal array (def: 1)" },
      },

      ports = {
	 { name="out", out_type_name="double", out_data_len=1, doc="controller output" },
      },

      operations = { start=true, stop=true, step=true }
}
