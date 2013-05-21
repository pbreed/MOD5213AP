PLATFORM=MOD5213
# This is a minimal make file. Anything that starts with a # is a comment.
#
# To generate the dependancies automatically, run "make depend"
#
# To clean up the directory run "make clean".
#
# Run "make depend" whenever:
#	You add files to the project
#	You change what files are included in a source file
#
# Run "make clean" whenever you change this makefile.
#
# Setup the project root name.
# This will build NAME.x and save it as $(NBROOT)/bin/NAME.x
NAME	= MOD5213AP
#CSRCS   := main.c
#Uncomment and modify these lines if you have CPP or S files.
CXXSRCS := main.cpp i2c_sub.cpp pitr_sem.cpp servo_drive.cpp spi_sub.cpp a2d_sub.cpp log.cpp sensor_config.cpp imu.cpp exmath.cpp dsm2_sub.cpp ublxgps.cpp
#ASRCS := foo.s

#include the file that does all of the automagic work!
include $(NBROOT)/make/main.mak


