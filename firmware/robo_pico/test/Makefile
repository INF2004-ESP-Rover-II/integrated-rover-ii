# Laptop unit tests for the Motor Control logic (Subsystem 6).
#   make        build and run
#   make clean
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wconversion -Wno-sign-conversion -O1 -g
INC      = -I../include

.PHONY: check check_wifi route goto dry
check: test_motor_control test_motor_front_stop check_wifi route goto dry

test_motor_front_stop: test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c
	$(CC) $(CFLAGS) -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1 -Istubs $(INC) -o $@ test_motor_front_stop.c ../src/motor_driver.c ../src/motor_control.c ../src/safety_interlock.c -lm
	./$@

test_motor_control: test_motor_control.c ../src/motor_control.c ../include/motor_control.h ../include/motor_types.h ../include/motor_config.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_motor_control.c ../src/motor_control.c -lm
	./$@

# Subsystem 4: A* planner and GOTO controller (uses B1's real rover_map.c and fixture; no Pico SDK)
route: test_route_planner
	./$<
goto: test_goto_ctrl
	./$<
dry: test_goto_dry
	./$<
test_goto_dry: test_goto_dry.c ../src/goto_dry.c ../src/goto_ctrl.c ../src/route_planner.c $(FIX)
	$(CC) -std=c11 -Wall -Wextra -O1 -g $(INC) -Ifixtures -o $@ test_goto_dry.c ../src/goto_dry.c ../src/goto_ctrl.c ../src/route_planner.c $(FIX)
FIX = fixtures/planner_fixture.c ../src/rover_map.c
test_route_planner: test_route_planner.c ../src/route_planner.c ../include/route_planner.h $(FIX)
	$(CC) -std=c11 -Wall -Wextra -O1 -g $(INC) -Ifixtures -o $@ test_route_planner.c ../src/route_planner.c $(FIX)
test_goto_ctrl: test_goto_ctrl.c ../src/goto_ctrl.c ../src/route_planner.c ../include/goto_ctrl.h $(FIX)
	$(CC) -std=c11 -Wall -Wextra -O1 -g $(INC) -Ifixtures -o $@ test_goto_ctrl.c ../src/goto_ctrl.c ../src/route_planner.c $(FIX)

clean:
	rm -f test_motor_control test_motor_front_stop test_wifi_mqtt test_route_planner test_goto_ctrl test_goto_dry

check_wifi: test_wifi_mqtt
	./$<

test_wifi_mqtt: test_wifi_mqtt.c ../src/wifi_mqtt.c ../include/wifi_mqtt.h wifi_stubs/wifi_test_sdk.h
	$(CC) $(CFLAGS) -Wno-unused-parameter -Iwifi_stubs $(INC) -o $@ test_wifi_mqtt.c ../src/wifi_mqtt.c
