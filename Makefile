CPPFLAGS = -std=c++14 -DUNIT_LIB_DISABLE_IOSTREAM -DDISABLE_PREDEFINED_UNITS -DENABLE_PREDEFINED_VOLTAGE_UNITS -DENABLE_PREDEFINED_TIME_UNITS -DENABLE_PREDEFINED_FREQUENCY_UNITS -Wno-deprecated-declarations -Wno-unused-function
CPPFLAGS += $(shell pkg-config --cflags lilv-0)
LDLIBS = -lbrlapi -lboost_serialization
LDLIBS += $(shell pkg-config --libs lilv-0)

pepper: render.cpp DSP.h
	$(MAKE) -C ../.. PROJECT=pepper CPPFLAGS="$(CPPFLAGS)" LDLIBS="$(LDLIBS)"
