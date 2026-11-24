# Laptop unit tests for the Motor Control logic (Subsystem 6).
#   make        build and run
#   make clean
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wconversion -Wno-sign-conversion -O1 -g
INC      = -I../include

.PHONY: check check_wifi route goto dry dfs barrc
check: test_motor_control test_motor_front_stop check_wifi route goto dry dfs

test_motor_front_stop: test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c
	$(CC) $(CFLAGS) -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1 -Istubs $(INC) -o $@ test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c -lm
	./$@

test_motor_control: test_motor_control.c ../src/motor_control.c ../include/motor_control.h ../include/motor_types.h ../include/motor_config.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_motor_control.c ../src/motor_control.c -lm
	./$@

# Subsystem 4: route planner, GOTO controller and DFS backtracking (BARR-C:2018 style, C99).
# Uses Buddy 1's real rover_map.c and fixture; no Pico SDK.
S4FLAGS = -std=c99 -pedantic -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wcast-qual -Wstrict-prototypes -O1 -g
FIX     = fixtures/planner_fixture.c ../src/rover_map.c
SUPPORT = test_support.c

route: test_route_planner
	./$<
goto: test_goto_ctrl
	./$<
dry: test_goto_dry
	./$<
dfs: test_dfs_backtrack
	./$<
barrc:
	python3 ../tools/barrc_check.py ../src/route_planner.c ../src/goto_ctrl.c ../src/goto_dry.c \
		../include/route_planner.h ../include/goto_ctrl.h ../include/goto_dry.h \
		test_route_planner.c test_goto_ctrl.c test_goto_dry.c test_dfs_backtrack.c test_support.c test_support.h

test_route_planner: test_route_planner.c $(SUPPORT) ../src/route_planner.c ../include/route_planner.h $(FIX)
	$(CC) $(S4FLAGS) $(INC) -Ifixtures -I. -o $@ test_route_planner.c $(SUPPORT) ../src/route_planner.c $(FIX)
test_goto_ctrl: test_goto_ctrl.c $(SUPPORT) ../src/goto_ctrl.c ../src/route_planner.c ../include/goto_ctrl.h $(FIX)
	$(CC) $(S4FLAGS) $(INC) -Ifixtures -I. -o $@ test_goto_ctrl.c $(SUPPORT) ../src/goto_ctrl.c ../src/route_planner.c $(FIX)
test_goto_dry: test_goto_dry.c $(SUPPORT) ../src/goto_dry.c ../src/goto_ctrl.c ../src/route_planner.c $(FIX)
	$(CC) $(S4FLAGS) $(INC) -Ifixtures -I. -o $@ test_goto_dry.c $(SUPPORT) ../src/goto_dry.c ../src/goto_ctrl.c ../src/route_planner.c $(FIX)
test_dfs_backtrack: test_dfs_backtrack.c $(SUPPORT) ../src/route_planner.c ../src/rover_map.c
	$(CC) $(S4FLAGS) $(INC) -I. -o $@ test_dfs_backtrack.c $(SUPPORT) ../src/route_planner.c ../src/rover_map.c

clean:
	rm -f test_motor_control test_motor_front_stop test_wifi_mqtt test_route_planner test_goto_ctrl test_goto_dry test_dfs_backtrack

check_wifi: test_wifi_mqtt
	./$<

test_wifi_mqtt: test_wifi_mqtt.c ../src/wifi_mqtt.c ../include/wifi_mqtt.h wifi_stubs/wifi_test_sdk.h
	$(CC) $(CFLAGS) -Wno-unused-parameter -Iwifi_stubs $(INC) -o $@ test_wifi_mqtt.c ../src/wifi_mqtt.c
