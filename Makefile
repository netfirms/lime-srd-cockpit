# Convenience Makefile for building and running LimeSrdCockpit

APP_NAME = LimeSrdCockpit
APP_BUNDLE = $(APP_NAME).app
APP_BINARY = $(APP_BUNDLE)/Contents/MacOS/$(APP_NAME)

.PHONY: all clean run

all: Makefile.qmake
	$(MAKE) -f Makefile.qmake

Makefile.qmake: LimeSrdCockpit.pro
	qmake -o Makefile.qmake LimeSrdCockpit.pro

clean:
	@if [ -f Makefile.qmake ]; then \
		$(MAKE) -f Makefile.qmake clean || true; \
		rm -f Makefile.qmake; \
	fi
	rm -rf *.o moc_* ui_* qrc_* $(APP_BUNDLE)
	rm -rf gnss_sdr_limesdr_temp.conf limesdr_fifo.dat

run: all
	./$(APP_BINARY)
