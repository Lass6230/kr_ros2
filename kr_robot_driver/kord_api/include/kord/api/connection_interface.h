
#include "kr2/kord/protocol/KORDFrames.h"

// #include <boost/asio.hpp>
#include "asio.hpp"

#include <memory>
#include <string>

using asio::ip::tcp;
using asio::ip::udp;

namespace kr2::kord {

enum connection { TCP_SERVER, TCP_CLIENT, UDP_SERVER, UDP_CLIENT };

class Connection_interface {
public:
    virtual bool connect(const char *) = 0;

    virtual bool disconnect() = 0;

    virtual unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *a_frame) = 0;

    virtual unsigned int recvFrame(kr2::kord::protocol::KORDFrame *a_frame) = 0;

    // check if an interface is still connected
    virtual bool is_connected() = 0;
};

class TCP_client : public Connection_interface {
    tcp::socket sock_;
    tcp::resolver resolver_;
    asio::io_service io_service_;

    std::string hostname_;

    unsigned short port_;

public:
    // initialisation of sock and resolver
    TCP_client(asio::io_service &io_service, std::string a_hostname, unsigned short a_port);

    bool connect(const char *device) override;

    bool disconnect() override;

    unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *a_frame) override;

    unsigned int recvFrame(kr2::kord::protocol::KORDFrame *a_frame) override;

    bool is_connected() override;
};

class TCP_server : public Connection_interface {
    tcp::acceptor acceptor_;
    asio::io_service io_service_;
    tcp::socket sock_;

public:
    TCP_server(asio::io_service &io_service, unsigned short port);

    bool connect(const char *device) override;

    bool disconnect() override;

    unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *a_frame) override;

    unsigned int recvFrame(kr2::kord::protocol::KORDFrame *a_frame) override;

    bool is_connected() override;
};

class UDP_client : public Connection_interface {
    udp::socket sock_;

    udp::resolver resolver_;
    std::string hostname_;

    // udp::resolver::results_type endpoints_;
    asio::ip::basic_resolver_iterator<asio::ip::udp> endpoints_;

    udp::endpoint sender_endpoint_;

    // unsigned short port_;

public:
    UDP_client(asio::io_service &io_service, std::string a_hostname, unsigned short a_port);

    bool connect(const char *device) override;

    bool disconnect() override;

    unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *a_frame) override;

    unsigned int recvFrame(kr2::kord::protocol::KORDFrame *a_frame) override;

    bool is_connected() override;
};

class UDP_server : public Connection_interface {

    udp::socket sock_;

    asio::io_service io_service_;

    udp::resolver resolver_;
    std::string hostname_;

    asio::ip::basic_resolver_iterator<asio::ip::udp> endpoints_;

    udp::endpoint sender_endpoint_;

public:
    UDP_server(asio::io_service &io_service, std::string a_hostname, unsigned short a_port);

    bool connect(const char *device) override;

    bool disconnect() override;

    unsigned int sendFrame(const kr2::kord::protocol::KORDFrame *a_frame) override;

    unsigned int recvFrame(kr2::kord::protocol::KORDFrame *a_frame) override;

    bool is_connected() override;
};

} // namespace kr2::kord
