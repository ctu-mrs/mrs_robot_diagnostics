#pragma once
#include <cstdint>
#include "mrs_robot_diagnostics/enums/enum_helpers.h"

#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME       uav_state_t
#define X_ENUM_BASE_TYPE  uint8_t
#define X_ENUM_SEQ                      \
                          (DISARMED)    \
                          (ARMED)       \
                          (MANUAL)      \
                          (TAKEOFF)     \
                          (LAND)        \
                          (RC_MODE)     \
                          (HOVER)       \
                          (GOTO)        \
                          (TRAJECTORY)

namespace mrs_robot_diagnostics
{

DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

}
