for OS that use the KS DLC board
At this point is for the Tube Rotator and Orbital Shaker( not custom board)

([A-Z])(([A-Z])*)  
\U$1\L$2 eplace constants like AAA_BBB with AssBbb

then
_([A-Z][a-z])
$1


then in main.cpp
([A-Z])([A-Z]*)_([A-Z])([A-Z]*)
$1\L$2$3\L$4

esptool chip-id
chip type:          ESP32-D0WD (revision v1.0)
Features:           Wi-Fi, BT, Dual Core + LP Core, 240MHz, Vref calibration in eFuse, Coding Scheme None
Crystal frequency:  40MHz
