# Laptop unit tests for the Motor Control logic (Subsystem 6).
#   make        build and run
#   make clean
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wconversion -Wno-sign-conversion -O1 -g
INC      = -I../include

.PHONY: check
check: test_motor_control test_motor_front_stop map

test_motor_front_stop: test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c
	$(CC) $(CFLAGS) -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1 -Istubs $(INC) -o $@ test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c -lm
	./$@

test_motor_control: test_motor_control.c ../src/motor_control.c ../include/motor_control.h ../include/motor_types.h ../include/motor_config.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_motor_control.c ../src/motor_control.c -lm
	./$@

# Subsystem 4 (mapping + A*): C port checked against the Python implementation, then the MAP_ commands.
#   make map                              uses the vectors in vectors/ (regenerate: see tools_gen_c_vectors.py)
MAPSRC = ../src/rover_map.c ../src/route_planner.c ../src/map_fsm.c ../src/map_cmd.c
.PHONY: map
map: vectors/map_vectors.txt
	$(CC) $(CFLAGS) -fsanitize=address,undefined $(INC) -o test_map_port test_map_port.c ../src/rover_map.c ../src/route_planner.c ../src/map_fsm.c
	./test_map_port vectors/map_vectors.txt
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -fsanitize=address,undefined $(INC) -o test_map_cmd test_map_cmd.c $(MAPSRC)
	./test_map_cmd

clean:
	rm -f test_motor_control test_motor_front_stop test_map_port test_map_cmd
