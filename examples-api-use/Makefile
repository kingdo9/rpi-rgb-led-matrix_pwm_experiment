include ../config.mk

CFLAGS=-Wall -O3 $(CPU_ARCH_FLAGS) $(LTO_FLAGS) -g -Wextra -Wno-unused-parameter
CXXFLAGS=$(CFLAGS)
SPWM_REGISTER_TEST_OBJECT=spwm-register-test.o
OBJECTS=demo-main.o $(SPWM_REGISTER_TEST_OBJECT) minimal-example.o c-example.o text-example.o scrolling-text-example.o clock.o ledcat.o input-example.o pixel-mover.o
BINARIES=demo minimal-example c-example text-example scrolling-text-example clock ledcat input-example pixel-mover

# Where our library resides. You mostly only need to change the
# RGB_LIB_DISTRIBUTION, this is where the library is checked out.
RGB_LIB_DISTRIBUTION=..
RGB_INCDIR=$(RGB_LIB_DISTRIBUTION)/include
RGB_LIBDIR=$(RGB_LIB_DISTRIBUTION)/lib
RGB_LIBRARY_NAME=rgbmatrix
RGB_LIBRARY=$(RGB_LIBDIR)/lib$(RGB_LIBRARY_NAME).a
SPWM_REGISTER_TEST_DIR=$(RGB_LIBDIR)/spwm/registertest
LDFLAGS+=$(CPU_ARCH_FLAGS) $(LTO_FLAGS) -L$(RGB_LIBDIR) -l$(RGB_LIBRARY_NAME) -lrt -lm -lpthread

# To compile image-example
MAGICK_CXXFLAGS?=$(shell GraphicsMagick++-config --cppflags --cxxflags)
MAGICK_LDFLAGS?=$(shell GraphicsMagick++-config --ldflags --libs)

all : $(BINARIES)

$(RGB_LIBRARY): FORCE
	$(MAKE) -C $(RGB_LIBDIR)

# Demo 15 uses internal refresh-thread profile hooks, but its interactive UI
# object stays linked only into the demo binary rather than the core library.
demo : demo-main.o $(SPWM_REGISTER_TEST_OBJECT) $(RGB_LIBRARY)
	$(CXX) demo-main.o $(SPWM_REGISTER_TEST_OBJECT) -o $@ $(LDFLAGS)

demo-main.o: $(SPWM_REGISTER_TEST_DIR)/spwm-register-test.h
demo-main.o: CXXFLAGS+=-I$(RGB_LIBDIR)

$(SPWM_REGISTER_TEST_OBJECT): \
	$(SPWM_REGISTER_TEST_DIR)/spwm-register-test.cc \
	$(SPWM_REGISTER_TEST_DIR)/spwm-register-test.h \
	$(SPWM_REGISTER_TEST_DIR)/spwm-register-profile-loader.h \
	$(RGB_INCDIR)/graphics.h \
	$(RGB_INCDIR)/led-matrix.h \
	$(RGB_LIBDIR)/spwm-helpers.h
	$(CXX) -I$(RGB_INCDIR) -I$(RGB_LIBDIR) $(CXXFLAGS) -c -o $@ $<

minimal-example : minimal-example.o
input-example : input-example.o
text-example: text-example.o
scrolling-text-example : scrolling-text-example.o
clock : clock.o
ledcat : ledcat.o
pixel-mover : pixel-mover.o

# All the binaries that have the same name as the object file.q
% : %.o $(RGB_LIBRARY)
	$(CXX) $< -o $@ $(LDFLAGS)

image-example.o : image-example.cc
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

image-example: image-example.o $(RGB_LIBRARY)
	$(CXX) $< -o $@ $(LDFLAGS) $(MAGICK_LDFLAGS)

# Since the C example uses the C++ library underneath, which depends on C++
# runtime stuff, you still have to also link -lstdc++
c-example : c-example.o $(RGB_LIBRARY)
	$(CC) $< -o $@ $(LDFLAGS) -lstdc++

%.o : %.cc
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) -c -o $@ $<

%.o : %.c
	$(CC) -I$(RGB_INCDIR) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(BINARIES)

FORCE:
.PHONY: FORCE
