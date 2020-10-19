ubxmoddir = ${UBX_MODDIR}
ubxmod_LTLIBRARIES = graphics_bridge.la



CLEANFILES = 

graphics_bridge_la_SOURCES = graphics_bridge.cpp
graphics_bridge_la_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@ -lmeshcatcpp -lzmq -liitgenur5
graphics_bridge_la_CPPFLAGS = -I${top_srcdir}/libubx @UBX_CFLAGS@ -fvisibility=hidden
%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

