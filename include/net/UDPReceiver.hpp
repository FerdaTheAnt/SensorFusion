#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>

class UDPReceiver {
public:
    using Callback = std::function<void(const std::string&)>;

    UDPReceiver(int port, Callback cb);
    ~UDPReceiver();

    void start();
    void stop();

private:
    void receiveLoop();

    int port_;
    std::thread recv_thread_;
    std::atomic<bool> running_;
    Callback callback_;
};
