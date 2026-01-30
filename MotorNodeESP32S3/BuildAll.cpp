// BuildAll.cpp
// Forces Arduino to compile and link all .cpp files that live in subfolders.
// DO NOT include headers here — include the .cpp files directly.

#include "comms/Protocol.cpp"
#include "comms/EspNowManager.cpp"

#include "config/ConfigStore.cpp"

#include "drivers/TMC2209Driver.cpp"

#include "motion/StepLossMonitor.cpp"
#include "motion/MotionController.cpp"

#include "motion_engine/MotionEngine_FAS.cpp"

#include "sensors/EncoderAS5047P.cpp"
#include "sensors/Thermistor.cpp"

#include "system/Tasks.cpp"
