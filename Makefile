all: ModernPsoptInterface.a

TARGET := ModernPsoptInterface

OBJECT_DEPS =

CXXFLAGSEXTRA = -std=c++0x -pedantic

include $(PSOPT)/Makefile_include.mk

install: ModernPsoptInterface.a
	cp ModernPsoptInterface.a $(PREFIX)/lib/
	cp ModernPsoptInterface.h $(PREFIX)/include/

ModernPsoptInterface.a: ModernPsoptInterface.o
	ar rcs  $@ $^

projectclean:
	rm -f ModernPsoptInterface.a
