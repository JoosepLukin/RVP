#pragma once

// Arduino IDE auto-generates prototypes for functions in .ino files.
// If those prototypes mention types declared later in the sketch, the build can fail.
// Keeping forward declarations in a header included at the very top avoids that.

#include <Arduino.h>

struct MsgCommand;
struct MsgAck;
struct MsgStatus;

struct MotorRecord;
struct RxItem;
struct TxQueued;
struct PendingTx;

