ubxmoddir = ${UBX_MODDIR}
ubxmod_LTLIBRARIES = signal_generator.la



CLEANFILES = 

signal_generator_la_SOURCES = signal_generator.c
signal_generator_la_LDFLAGS = -module -avoid-version -shared -export-dynamic @UBX_LIBS@
signal_generator_la_CFLAGS = -I${top_srcdir}/libubx @UBX_CFLAGS@ -fvisibility=hidden
%.h.hexarr: %.h
	ubx-tocarr -s $< -d $<.hexarr

