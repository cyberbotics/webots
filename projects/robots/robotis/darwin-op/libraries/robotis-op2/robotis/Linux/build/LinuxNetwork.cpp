/*
 *   LinuxNetwork.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <iostream>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include "LinuxNetwork.h"

using namespace Robot;
using namespace std;


LinuxSocket::LinuxSocket() : m_sock ( -1 )
{
    memset ( &m_addr, 0, sizeof ( m_addr ) );
    m_non_blocking = false;
}

LinuxSocket::~LinuxSocket()
{
    if ( is_valid() )
        ::close ( m_sock );
}

bool LinuxSocket::create()
{
    m_sock = socket ( AF_INET,
                      SOCK_STREAM,
                      0 );

    if ( ! is_valid() )
        return false;

    // TIME_WAIT - argh
    int on = 1;
    if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
        return false;

    return true;
}

bool LinuxSocket::bind ( const int port )
{
    if ( ! is_valid() )
    {
        return false;
    }

    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = INADDR_ANY;
    m_addr.sin_port = htons ( port );

    int bind_return = ::bind ( m_sock,
                             ( struct sockaddr * ) &m_addr,
                             sizeof ( m_addr ) );
    if ( bind_return == -1 )
    {
        return false;
    }

    return true;
}
bool LinuxSocket::listen() const
{
    if ( ! is_valid() )
    {
        return false;
    }

    int listen_return = ::listen ( m_sock, MAXCONNECTIONS );

    if ( listen_return == -1 )
    {
        return false;
    }
    return true;
}

bool LinuxSocket::accept ( LinuxSocket& new_socket ) const
{
    int addr_length = sizeof ( m_addr );

    errno = 0;
    new_socket.m_sock = ::accept ( m_sock, ( sockaddr * ) &m_addr, ( socklen_t * ) &addr_length );

    if ( m_non_blocking && ( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) ) )
    {
        return false; // don't block
    }
    else if ( new_socket.m_sock <= 0 )
        return false;
    else
        return true;
}

bool LinuxSocket::send ( const std::string s ) const
{
    errno = 0;
    int status = ::send ( m_sock, s.c_str(), s.size(), MSG_NOSIGNAL | ( m_non_blocking ? MSG_DONTWAIT : 0 ) );

    if ( ( status >= 0 ) || ( m_non_blocking  && ( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) ) ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool LinuxSocket::send ( void* data, int length ) const
{
    errno = 0;
    int status = ::send ( m_sock, data, length, MSG_NOSIGNAL );
    if ( ( status >= 0 ) || ( m_non_blocking  && ( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) ) ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

int LinuxSocket::recv ( std::string& s ) const
{
    char buf [ MAXRECV + 1 ];

    s = "";

    memset ( buf, 0, MAXRECV + 1 );

    errno = 0;
    int status = ::recv ( m_sock, buf, MAXRECV, m_non_blocking ? MSG_DONTWAIT : 0 );

    if ( status == 0 )
    {
        return 0; // connection was gracefully closed
    }
    else if ( status == -1 )
    {
        if ( m_non_blocking  && ( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) ) )
        {
            return 1; // true result
        }
        else
        {
            cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
            return 0;
        }
    }
    else
    {
        s = string(buf,status);
        return status;
    }
}

int LinuxSocket::recv ( void* data, int length ) const
{
    errno = 0;
    int status = ::recv ( m_sock, data, length, m_non_blocking ? MSG_DONTWAIT : 0 );

    if ( status == 0 )
    {
        return 0; // connection was gracefully closed
    }
    else if ( status == -1 )
    {
        if ( m_non_blocking  && ( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) ) )
        {
            return 0; // no data to return
        }
        else
        {
            cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
            return 0;
        }
    }
    
    return status;
}

bool LinuxSocket::connect ( const std::string host, const int port )
{
    if ( ! is_valid() ) return false;

    m_addr.sin_family = AF_INET;
    m_addr.sin_port = htons ( port );

    int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );

    if ( errno == EAFNOSUPPORT ) return false;

    status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );

    if ( status == 0 )
        return true;
    else
        return false;
}

void LinuxSocket::set_non_blocking ( const bool b )
{
    int opts;

    opts = fcntl ( m_sock, F_GETFL );

    if ( opts < 0 )
    {
        return;
    }

    if ( b )
        opts = ( opts | O_NONBLOCK );
    else
        opts = ( opts & ~O_NONBLOCK );

    fcntl ( m_sock,
            F_SETFL,opts );

    m_non_blocking = b;
}

const LinuxSocket& LinuxSocket::operator << ( const std::string& s ) const
{	
    if ( ! send ( s ) )
    {
        throw LinuxSocketException ( "Could not write to socket." );
    }

    return *this;
}

const LinuxSocket& LinuxSocket::operator << ( const int& i ) const
{
    std::stringstream ss;
    ss << i;

    if ( ! send ( ss.str() ) )
    {
        throw LinuxSocketException ( "Could not write to socket." );
    }

    return *this;
}

const LinuxSocket& LinuxSocket::operator >> ( std::string& s ) const
{
    if ( ! recv ( s ) )
    {
        throw LinuxSocketException ( "Could not read from socket." );
    }

    return *this;
}

LinuxServer::LinuxServer ( int port )
{
    listen(port);
}

LinuxServer::~LinuxServer()
{
}

void LinuxServer::listen ( int port )
{
    if ( ! LinuxSocket::create() )
    {
        throw LinuxSocketException ( "Could not create server socket." );
    }

    if ( ! LinuxSocket::bind ( port ) )
    {
        throw LinuxSocketException ( "Could not bind to port." );
    }

    if ( ! LinuxSocket::listen() )
    {
        throw LinuxSocketException ( "Could not listen to socket." );
    }
}


