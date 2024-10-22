#include <kord/api/connection_interface.h>
//#include <boost/asio.hpp>
#include <asio.hpp>

#include <memory>
#include <iostream>
#ifdef __APPLE__
#include <kord/utils/endian.h>
#endif
using namespace kr2::kord;
using namespace kr2::kord::protocol;

/*#############################################################################*/
/*############################ TCP_client #####################################*/
/*#############################################################################*/

TCP_client::TCP_client (asio::io_service& io_service, 
                        std::string a_hostname, 
                        unsigned short a_port):
                        sock_ (io_service_), 
                        resolver_(io_service_),
                        hostname_(a_hostname), 
                        port_(a_port)
{}

bool TCP_client::connect(const char * device){
    std::stringstream ss;
    ss << port_;
    tcp::resolver::query q(hostname_, ss.str());
    asio::connect(sock_, resolver_.resolve(q));
    // TODO: specify device endpoint for the interface 

    if ( ! sock_.is_open() ){
        std::cout << "Connection was not established." << std::endl;
    }

    return true;
}

bool TCP_client::disconnect(){
    asio::error_code ec;
    sock_.close(ec);
    if (ec)
    {
        // An error occurred.
        return false;
    }
    return true;
}

unsigned int TCP_client::sendFrame(const KORDFrame * a_frame){
    return asio::write(sock_, asio::buffer(a_frame, sizeof(KORDFrame)));
};

unsigned int TCP_client::recvFrame(KORDFrame * a_frame){
    try {
    return asio::read(sock_, asio::buffer(a_frame, sizeof(KORDFrame)));
    } catch(asio::system_error& e) {
        return -1;
    }
}

bool TCP_client::is_connected(){
    return sock_.is_open();
}

/*#############################################################################*/
/*############################ TCP_server #####################################*/
/*#############################################################################*/

TCP_server::TCP_server( asio::io_service& io_service, 
                        unsigned short port ):
                        acceptor_(io_service_, 
                        tcp::endpoint(tcp::v4(), port)), 
                        sock_(io_service_)
{}

bool TCP_server::connect(const char * device){

    int native_socket = sock_.native_handle();

    timeval rcv_timeout{0, 10000};
    setsockopt(native_socket, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));


    // TODO: specify endpoints for acceptor
    acceptor_.accept(sock_);
    if ( ! sock_.is_open() ) {
        return false;
    }
    return true;
}

bool TCP_server::disconnect(){
    asio::error_code ec;
    sock_.close(ec);
    if (ec)
    {
        // An error occurred.
        return false;
    }
    return true;
}

unsigned int TCP_server::sendFrame(const KORDFrame * a_frame){
    return asio::write(sock_, asio::buffer(a_frame, sizeof(KORDFrame)));
}

unsigned int TCP_server::recvFrame(KORDFrame * a_frame){
    try{
        return asio::read(sock_, asio::buffer(a_frame, sizeof(KORDFrame)));
    } catch(asio::system_error& e) {
        return -1;
    }
}

bool TCP_server::is_connected(){
    return sock_.is_open();
}

/*#############################################################################*/
/*############################ UDP_client #####################################*/
/*#############################################################################*/

UDP_client::UDP_client( asio::io_service& io_service, 
                        std::string a_hostname, 
                        unsigned short a_port):
                        sock_ (io_service, udp::endpoint(udp::v4(), a_port)), 
                        resolver_(io_service),
                        hostname_(a_hostname)
{

    std::stringstream ss;
    ss << a_port;
    //endpoints_ = resolver_.resolve(udp::v4(), a_hostname, ss.str());
    udp::resolver::query q(udp::v4(), a_hostname, ss.str());
    endpoints_ = resolver_.resolve(q);
}

bool UDP_client::connect(const char * device){

    struct hostent *hp;
    asio::error_code ec;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    // TODO: refactor the magic constans
    int native_socket = sock_.native_handle();

    timeval rcv_timeout{0, 10};
    setsockopt(native_socket, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

    if (device){
        snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", device);
        // boost cannot set rcv_timeout so native_handle will return native socket representation that provides it

#ifdef __APPLE__
        int idx = if_nametoindex(device);
        if (setsockopt(native_socket, IPPROTO_IP, IP_BOUND_IF, &idx, sizeof(idx)) < 0) {
            std::cout << "Unable to bind " << device <<  " to the socket." << std::endl;
            return false;
        }
#else
        if (setsockopt(native_socket, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr)) < 0) {
            std::cout << "Unable to bind " << device <<  " to the socket." << std::endl;
            return false;
        }
#endif
    }


    hp = (struct hostent *)gethostbyname(hostname_.c_str());
    if (!hp) {
        std::cout << "Unknown host\n";
        sock_.close(ec);
        if (ec) {
            // An error occurred
        }
        return false;
    }

    return true;
}

bool UDP_client::disconnect(){
    asio::error_code ec;
    sock_.close(ec);
    if (ec)
    {
        // An error occurred.
        return false;
    }
    return true;
}

unsigned int UDP_client::sendFrame(const KORDFrame * a_frame){
    return sock_.send_to(asio::buffer(a_frame, sizeof(KORDFrame)), *endpoints_);
}
// old implementation -> interupt didn't work
// unsigned int UDP_client::recvFrame(KORDFrame * a_frame){
//     int n = 0;
//     try { 
//         n = sock_.receive_from(asio::buffer(a_frame, sizeof(KORDFrame)), sender_endpoint_);
//         // convert from BE ordering to host ordering
//         a_frame->frame_id_ = ntohs(a_frame->frame_id_);
//         a_frame->session_id_ = ntohs(a_frame->session_id_);
//         a_frame->tx_timestamp_ = be64toh(a_frame->tx_timestamp_);
//         a_frame->kord_version_ = ntohs(a_frame->kord_version_);
//         a_frame->payload_length_ = ntohs(a_frame->payload_length_);
//     } 
//     catch(asio::system_error& e)  { return 0; }
//     // Ctrl + C case from outside

//     return n;
// }

unsigned int UDP_client::recvFrame(KORDFrame * a_frame) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(sock_.native_handle(), &read_set);

    struct timeval timeout;
    timeout.tv_sec = 0; // 2 ms timeout
    timeout.tv_usec = 4000;

    int result = select(sock_.native_handle() + 1, &read_set, nullptr, nullptr, &timeout);

    if (result > 0) {
        if (FD_ISSET(sock_.native_handle(), &read_set)) {
            int n = sock_.receive_from(asio::buffer(a_frame, sizeof(KORDFrame)), sender_endpoint_);
            // convert from BE ordering to host ordering
            a_frame->frame_id_ = ntohs(a_frame->frame_id_);
            a_frame->session_id_ = ntohs(a_frame->session_id_);
            a_frame->tx_timestamp_ = be64toh(a_frame->tx_timestamp_);
            a_frame->kord_version_ = ntohs(a_frame->kord_version_);
            a_frame->payload_length_ = ntohs(a_frame->payload_length_);
            return n;
        }
    } else if (result == 0) {
        // Timeout occurred
        //std::cout << "Receive timeout" << std::endl;
        return 0; // Return 0 to indicate a timeout
    } else {
        // An error occurred
        //std::cerr << "Error in select: " << strerror(errno) << std::endl;
        return -1; // Return -1 to indicate an error
    }

    return -1; // Return -1 for unknown error
}



bool UDP_client::is_connected(){
    return sock_.is_open();
}

/*#############################################################################*/
/*############################ UDP_server #####################################*/
/*#############################################################################*/

UDP_server::UDP_server( asio::io_service& io_service,
                        std::string a_hostname, 
                        unsigned short a_port):
                        sock_ (io_service_, udp::endpoint(udp::v4(), a_port)), 
                        resolver_(io_service_),
                        hostname_(a_hostname)
{

    std::stringstream ss;
    ss << a_port;

    udp::resolver::query q(udp::v4(), hostname_, ss.str());
    endpoints_ = resolver_.resolve(q);
}

bool UDP_server::connect(const char * device){

    struct hostent *hp;
    asio::error_code ec;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    // TODO: refactor the magic constans
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", device);

    // boost cannot set rcv_timeout so native_handle will return native socket representation that provides it

    int native_socket = sock_.native_handle();
    timeval rcv_timeout{0, 10000};
    setsockopt(native_socket, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));
#ifdef __APPLE__
    int idx = if_nametoindex(device);
    if (setsockopt(native_socket, IPPROTO_IP, IP_BOUND_IF, &idx, sizeof(idx)) < 0) {
        std::cout << "Unable to bind ethnet to the socket." << std::endl;
            // TODO: pbbly return false
    }
#else
    if (setsockopt(native_socket, SOL_SOCKET, SO_BINDTODEVICE, (void *)&ifr, sizeof(ifr)) < 0) {
        std::cout << "Unable to bind ethnet to the socket." << std::endl;
        // TODO: pbbly return false
    }
#endif

    hp = (struct hostent *)gethostbyname(hostname_.c_str());
    if (!hp) {
        std::cout << "Unknown host\n";
        sock_.close(ec);
        if (ec) {
            // An error occured
        }
        return false;
    }
    else {
        std::cout << "Connection successful." << std::endl;
    }

    return true;
}

bool UDP_server::disconnect(){
    asio::error_code ec;
    sock_.close(ec);
    if (ec)
    {
        // An error occurred.
        return false;
    }
    return true;
}

unsigned int UDP_server::sendFrame(const KORDFrame * a_frame){
    return sock_.send_to(asio::buffer(a_frame, sizeof(KORDFrame)), *endpoints_);
}

unsigned int UDP_server::recvFrame(KORDFrame * a_frame){
    try{
        return sock_.receive_from(asio::buffer(a_frame, sizeof(KORDFrame)), sender_endpoint_);
    } catch(asio::system_error& e) {
        return -1;
    }
}
// unsigned int UDP_server::recvFrame(KORDFrame * a_frame) {
//     fd_set read_set;
//     FD_ZERO(&read_set);
//     FD_SET(sock_.native_handle(), &read_set);

//     struct timeval timeout;
//     timeout.tv_sec = 10; // 10 seconds timeout
//     timeout.tv_usec = 0;

//     int result = select(sock_.native_handle() + 1, &read_set, nullptr, nullptr, &timeout);

//     if (result > 0) {
//         if (FD_ISSET(sock_.native_handle(), &read_set)) {
//             return sock_.receive_from(boost::asio::buffer(a_frame, sizeof(KORDFrame)), sender_endpoint_);
//         }
//     } else if (result == 0) {
//         // Timeout occurred
//         std::cout << "Receive timeout" << std::endl;
//         return 0; // Return 0 to indicate a timeout
//     } else {
//         // An error occurred
//         std::cerr << "Error in select: " << strerror(errno) << std::endl;
//         return -1; // Return -1 to indicate an error
//     }

//     return -1; // Return -1 for unknown error
// }
bool UDP_server::is_connected(){
    return sock_.is_open();
}

