
MAKEFILE =	Makefile
DEL_FILE =    rm -f
CHK_DIR_EXISTS= test -d
MKDIR    = mkdir -p
INSTALL_FILE= 
INSTALL_DIR = 
SUBTARGETS =	 \
		sub-build

sub-build: build/$(MAKEFILE)
	cd build && $(MAKE) -f $(MAKEFILE)

%:
	cd build && $(MAKE) $@
