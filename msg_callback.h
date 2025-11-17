#pragma once
#include <cstddef>

class MsgCallback {
public:
    virtual void on_msg_callback(const void* msg, std::size_t msg_len) = 0;
};
