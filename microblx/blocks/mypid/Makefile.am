ubxmoddir = ${UBX_MODDIR}
ubxmod_LTLIBRARIES = mypid.la



CLEANFILES = 

mypid_la_SOURCES = mypid.c
mypid_la_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@
mypid_la_CFLAGS = -I${top_srcdir}/libubx @UBX_CFLAGS@ -fvisibility=hidden
%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

