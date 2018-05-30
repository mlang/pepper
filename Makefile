all:
	$(MAKE) -C ../.. PROJECT=pepper CPPFLAGS='-std=c++14 -Wno-deprecated-declarations -I/usr/include/lilv-0' LDLIBS='-llilv-0 -lbrlapi'
