// --specs=nosys.specs --specs=nano.specs no longer includes these stubs as of gcc-arm-embedded-12
// these are the minimal implementations that get libgloss/libnosys/warning.h to shut up
// we know these aren't used because -Wl,--gc-sections strips them clearing the error but leaving
// the warning. For some reasons, packages linker ignore -lnosys which should link these stubs but
// provided by the compiler. Others online are having this issue and the --gc-sections is the
// workaround to get the error to clear, and implementing the stubs is needed to get the warning
// to clear. These are valid implmentations, so won't cause an accidental usage of the newlib
// function to crash. In all liklihood, such a use would include a newlib function that has almost
// no chance of fitting in the flash memory.
//
// https://community.arm.com/support-forums/f/compilers-and-libraries-forum/53519/arm-gcc-11-3-rel1-newlib-nano-linker-warning-_read-is-not-implemented-and-will-always-fail
//

int _read(int file, char *ptr, int len) {
  return  0;
}

int _write(int file, char *buf, int nbytes) {        
  return 0;
}

int _lseek(int file, int offset, int whence) {
  return  0;
}

int _close (int file) {  
  return -1;
}