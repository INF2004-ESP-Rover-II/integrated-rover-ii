# Laptop unit tests for the Motor Control logic (Subsystem 6).
#   make        build and run
#   make clean
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wconversion -Wno-sign-conversion -O1 -g
INC      = -I../include

.PHONY: check
check: test_motor_control test_motor_front_stop

test_motor_front_stop: test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c
	$(CC) $(CFLAGS) -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1 -Istubs $(INC) -o $@ test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c -lm
	./$@

test_motor_control: test_motor_control.c ../src/motor_control.c ../include/motor_control.h ../include/motor_types.h ../include/motor_config.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_motor_control.c ../src/motor_control.c -lm
	./$@

clean:
	rm -f test_motor_control test_motor_front_stop
