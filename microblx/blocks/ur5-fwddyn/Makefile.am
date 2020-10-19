ubxmoddir = ${UBX_MODDIR}
ubxmod_LTLIBRARIES = robot_interface.la



CLEANFILES = 

robot_interface_la_SOURCES = robot_interface.cpp
robot_interface_la_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@ -liitgenur5
robot_interface_la_CPPFLAGS = -I${top_srcdir}/libubx @UBX_CFLAGS@ -fvisibility=hidden
%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

