/*
 *   LinuxNetwork.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _LINUX_NETWORK_H_
#define _LINUX_NETWORK_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>


namespace Robot
{
	class LinuxSocket
	{
	private:
		int m_sock;
		sockaddr_in m_addr;
		bool m_non_blocking;

	public:
		static const int MAXHOSTNAME = 200;
		static const int MAXCONNECTIONS = 5;
		static const int MAXRECV = 500;

		LinuxSocket();
		virtual ~LinuxSocket();

		// Server initialization
		bool create();
		bool bind ( const int port );
		bool listen() const;
		bool accept ( LinuxSocket& ) const;

		// Client initialization
		bool connect ( const std::string host, const int port );

		// Data Transimission
		bool send ( const std::string ) const;
		bool send ( void* data, int length ) const;
		int recv ( std::string& ) const;
		int recv ( void* data, int length ) const;

		void set_non_blocking ( const bool );

		bool is_valid() const { return m_sock != -1; }

		const LinuxSocket& operator << ( const std::string& ) const;
		const LinuxSocket& operator << ( const int& ) const;
		const LinuxSocket& operator >> ( std::string& ) const;	
	};

	class LinuxSocketException  
	{
	private:
		std::string m_s;

	public:
		LinuxSocketException ( std::string s ) : m_s ( s ) {};
		~LinuxSocketException (){};
	    
		std::string description() { return m_s; }	
	};

	class LinuxServer : public LinuxSocket
	{
	public:
		LinuxServer ( int port );
		LinuxServer () {};
		void listen ( int port );
		virtual ~LinuxServer();
	};
}

#endif
