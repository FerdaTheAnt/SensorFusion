#include "net/UDPReceiver.hpp"

#include <asio.hpp>
#include <iostream>

UDPReceiver::UDPReceiver(int port, Callback cb)
: port_(port), callback_(cb), running_(false) {}

UDPReceiver::~UDPReceiver() {
    stop();
}

void UDPReceiver::start() {
    running_ = true;
    recv_thread_ = std::thread(&UDPReceiver::receiveLoop, this);
}

void UDPReceiver::stop() {
    running_ = false;
    if(recv_thread_.joinable())
        recv_thread_.join();
}

void UDPReceiver::receiveLoop() {
    try {
        asio::io_context io_context;
        asio::ip::udp::socket socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), port_));
        char data[1024];

        while(running_){
            asio::ip::udp::endpoint sender_endpoint;
            size_t length = socket.receive_from(asio::buffer(data), sender_endpoint);

            std::string msg(data, length);
            callback_(msg);
        }
    } catch (std::exception& e){
        std::cerr << "[UDPReceiver] Error: " << e.what() << std::endl;
    }
}
