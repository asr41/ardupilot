#!/bin/bash

BOARD=Pixhawk6X

cat >/tmp/defaults.parm <<"EOF"
NET_ENABLE 1
NET_DHCP 0

NET_IPADDR0      169
NET_IPADDR1      254
NET_IPADDR2      55
NET_IPADDR3      70

SIM_OPOS_LAT      40.269712
SIM_OPOS_LNG      -73.769042
SIM_OPOS_ALT      33000
SIM_OPOS_HDG      0

SERVO1_FUNCTION  190
SERVO1_MAX       1580
SERVO1_MIN       1420
SERVO1_REVERSED  0
SERVO1_TRIM      1500
SERVO2_FUNCTION  191
SERVO2_MAX       1580
SERVO2_MIN       1400
SERVO2_REVERSED  0
SERVO2_TRIM      1450
SERVO3_FUNCTION  192
SERVO3_MAX       1600
SERVO3_MIN       1420
SERVO3_REVERSED  0
SERVO3_TRIM      1550
SERVO4_FUNCTION  -1
SERVO4_MAX       1900
SERVO4_MIN       800
SERVO4_REVERSED  0
SERVO4_TRIM      1000
SERVO6_FUNCTION  97

#REMOVE THIS FOR FLIGHT
env SIM_ENABLED 1

SIM_OH_MASK 63
SIM_OH_RELAY_MSK 63

#NET_OPTIONS 1
EOF

cat >/tmp/extra.hwdef <<"EOF"
define AP_SIM_JSON_ENABLED 1

define DISABLE_WATCHDOG 0
EOF

./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
--board $BOARD \
--vehicle plane \
--extra-hwdef=/tmp/extra.hwdef \
--defaults=/tmp/defaults.parm \
--debug

exit 0

#./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
#    --board $BOARD \
#    --vehicle plane \
#    --simclass JSON \
#    --frame json:169.254.55.69 \
#    --extra-hwdef=/tmp/extra.hwdef \
#    --defaults=/tmp/defaults.parm \
#    --debug
