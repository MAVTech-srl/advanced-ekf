#pragma once

enum SyncMsgStatus
{
    BUFFERS_EMPTY,      // Force or torque buffers are empty
    ONLY_INPUTS,        // Only input buffers are full
    MEASURMENTS         // Both input and measurement buffer are full
};