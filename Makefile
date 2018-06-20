CPPFLAGS = -std=c++14 -Wno-deprecated-declarations
CPPFLAGS += $(shell pkg-config --cflags lilv-0)
LDLIBS = -lbrlapi -lboost_serialization
LDLIBS += $(shell pkg-config --libs lilv-0)

all:
	$(MAKE) -C ../.. PROJECT=pepper CPPFLAGS="$(CPPFLAGS)" LDLIBS="$(LDLIBS)"
