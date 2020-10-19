ubxmoddir = ${UBX_MODDIR}
ubxmod_LTLIBRARIES = ext_time_trig.la



CLEANFILES = 

ext_time_trig_la_SOURCES = ext_time_trig.c
ext_time_trig_la_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@
ext_time_trig_la_CFLAGS = -I${top_srcdir}/libubx @UBX_CFLAGS@ -fvisibility=hidden
%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

