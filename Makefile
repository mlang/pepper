CPPFLAGS = -std=c++14 -Wno-deprecated-declarations -Wno-unused-function
CPPFLAGS += $(shell pkg-config --cflags lilv-0)
LDLIBS = -lbrlapi -lboost_serialization
LDLIBS += $(shell pkg-config --libs lilv-0)

pepper: render.cpp DSP.h
	$(MAKE) -C ../.. PROJECT=pepper CPPFLAGS="$(CPPFLAGS)" LDLIBS="$(LDLIBS)"
