AM_CFLAGS = -I$(top_srcdir)/libubx $(UBX_CFLAGS) -fvisibility=hidden -Wno-missing-field-initializers
AM_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@

ubxmoddir = ${UBX_MODDIR}

ubxmod_LTLIBRARIES = graphics_bridge.la \
		     signal_generator.la \
		     mypid.la \
		     ext_time_trig.la \
		     robot_interface.la

CLEANFILES = 

# graphics_bridge
graphics_bridge_la_SOURCES = blocks/gbridge/graphics_bridge.cpp
graphics_bridge_la_CPPFLAGS = ${AM_CFLAGS} $(EIGEN_CFLAGS)
graphics_bridge_la_LDFLAGS = ${AM_LDFLAGS} -lmeshcatcpp -lzmq -liitgenur5

# signal_generator
signal_generator_la_SOURCES = blocks/sgen/signal_generator.c
signal_generator_la_CFLAGS = ${AM_CFLAGS}
signal_generator_la_LDFLAGS = ${AM_LDFLAGS}

# mypid
mypid_la_SOURCES = blocks/mypid/mypid.c
mypid_la_CFLAGS = ${AM_CFLAGS}
mypid_la_LDFLAGS = ${AM_LDFLAGS}

# mytrig
ext_time_trig_la_SOURCES = blocks/mytrig/ext_time_trig.c
ext_time_trig_la_CFLAGS = ${AM_CFLAGS}
ext_time_trig_la_LDFLAGS = ${AM_LDFLAGS}

# robot_interface
robot_interface_la_SOURCES = blocks/ur5-fwddyn/robot_interface.cpp
robot_interface_la_CPPFLAGS = ${AM_CFLAGS} $(EIGEN_CFLAGS)
robot_interface_la_LDFLAGS = ${AM_LDFLAGS} -liitgenur5

%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

