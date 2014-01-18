all: libModernPsoptInterface.a

TARGET := ModernPsoptInterface

OBJECT_DEPS =

CXXFLAGSEXTRA = -std=c++0x -pedantic

include $(PSOPT)/Makefile_include.mk

install: libModernPsoptInterface.a
	cp libModernPsoptInterface.a $(PREFIX)/lib/
	cp ModernPsoptInterface.h $(PREFIX)/include/

ModernPsoptInterface.a: ModernPsoptInterface.o
	ar rcs  $@ $^

projectclean:
	rm -f libModernPsoptInterface.a
