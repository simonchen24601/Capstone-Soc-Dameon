#pragma once
#include <mutex>

template <typename T>
class SingletonT {
public:
    // Disable copy/move operations
    SingletonT(const SingletonT&) = delete;
    SingletonT& operator=(const SingletonT&) = delete;
    SingletonT(SingletonT&&) = delete;
    SingletonT& operator=(SingletonT&&) = delete;

    static T* get_instance() {
        static T instance;
        return &instance;
    }

protected:
    SingletonT() = default;
    virtual ~SingletonT() = default;
};
