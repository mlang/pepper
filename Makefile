BELA_HOME=~/Modular/Bela

build:
	$(BELA_HOME)/scripts/build_project.sh -m "CPPFLAGS='-I/usr/include/lilv-0' LDLIBS='-llilv-0'" $(CURDIR)
