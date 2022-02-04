/*
 *   LinuxCM730.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "LinuxCM730.h"

using namespace Robot;


LinuxCM730::LinuxCM730(const char* name)
{
    DEBUG_PRINT = false;
    m_Socket_fd = -1;
    m_PacketStartTime = 0;
    m_PacketWaitTime = 0;
    m_UpdateStartTime = 0;
    m_UpdateWaitTime = 0;
    m_ByteTransferTime = 0;

    sem_init(&m_LowSemID, 0, 1);
    sem_init(&m_MidSemID, 0, 1);
    sem_init(&m_HighSemID, 0, 1);

    SetPortName(name);
}

LinuxCM730::~LinuxCM730()
{
    ClosePort();
}

void LinuxCM730::SetPortName(const char* name)
{
    strcpy(m_PortName, name);
}

bool LinuxCM730::OpenPort()
{
    struct termios newtio;
    struct serial_struct serinfo;
    double baudrate = 1000000.0; //bps (1Mbps)
    
    ClosePort();

    if(DEBUG_PRINT == true)
        printf("\n%s open ", m_PortName);
    
    if((m_Socket_fd = open(m_PortName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
        goto UART_OPEN_ERROR;

    if(DEBUG_PRINT == true)
        printf("success!\n");

    // You must set 38400bps!
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    tcsetattr(m_Socket_fd, TCSANOW, &newtio);

    if(DEBUG_PRINT == true)
        printf("Set %.1fbps ", baudrate);

    // Set non-standard baudrate
    if(ioctl(m_Socket_fd, TIOCGSERIAL, &serinfo) < 0)
        goto UART_OPEN_ERROR;

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;
    
    if(ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0)
    {
        if(DEBUG_PRINT == true)
            printf("failed!\n");
        goto UART_OPEN_ERROR;
    }

    if(DEBUG_PRINT == true)
        printf("success!\n");

    tcflush(m_Socket_fd, TCIFLUSH);

    m_ByteTransferTime = (1000.0 / baudrate) * 12.0;
    
    return true;

UART_OPEN_ERROR:
    if(DEBUG_PRINT == true)
        printf("failed!\n");
    ClosePort();
    return false;
}

bool LinuxCM730::SetBaud(int baud)
{
    struct serial_struct serinfo;
    int baudrate = (int)(2000000.0f / (float)(baud + 1));

    if(m_Socket_fd == -1)
        return false;

    if(ioctl(m_Socket_fd, TIOCGSERIAL, &serinfo) < 0) {
        fprintf(stderr, "Cannot get serial info\n");
        return false;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if(ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        fprintf(stderr, "Cannot set serial info\n");
        return false;
    }

    ClosePort();
    OpenPort();

    m_ByteTransferTime = (float)((1000.0f / baudrate) * 12.0f * 8);

    return true;
}

void LinuxCM730::ClosePort()
{
    if(m_Socket_fd != -1)
        close(m_Socket_fd);
    m_Socket_fd = -1;
}

void LinuxCM730::ClearPort()
{
    tcflush(m_Socket_fd, TCIFLUSH);
}

int LinuxCM730::WritePort(unsigned char* packet, int numPacket)
{
    return write(m_Socket_fd, packet, numPacket);
}

int LinuxCM730::ReadPort(unsigned char* packet, int numPacket)
{
    return read(m_Socket_fd, packet, numPacket);
}

void sem_wait_nointr(sem_t *sem)
{
    int sem_result, sem_count = 0;
    do {
        sem_result = sem_wait(sem);
    } while((sem_result == -1) && (errno == EINTR));
}

void LinuxCM730::LowPriorityWait()
{
    sem_wait_nointr(&m_LowSemID);
}

void LinuxCM730::MidPriorityWait()
{
    sem_wait_nointr(&m_MidSemID);
}

void LinuxCM730::HighPriorityWait()
{
    sem_wait_nointr(&m_HighSemID);
}

void LinuxCM730::LowPriorityRelease()
{
    sem_post(&m_LowSemID);
}

void LinuxCM730::MidPriorityRelease()
{
    sem_post(&m_MidSemID);
}

void LinuxCM730::HighPriorityRelease()
{
    sem_post(&m_HighSemID);
}

double LinuxCM730::GetCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

void LinuxCM730::SetPacketTimeout(int lenPacket)
{
    m_PacketStartTime = GetCurrentTime();
    m_PacketWaitTime = m_ByteTransferTime * (double)lenPacket + 5.0;
}

bool LinuxCM730::IsPacketTimeout()
{
    if(GetPacketTime() > m_PacketWaitTime)
        return true;

    return false;
}

double LinuxCM730::GetPacketTime()
{
    double time;

    time = GetCurrentTime() - m_PacketStartTime;
    if(time < 0.0)
        m_PacketStartTime = GetCurrentTime();

    return time;
}

void LinuxCM730::SetUpdateTimeout(int msec)
{
    m_UpdateStartTime = GetCurrentTime();
    m_UpdateWaitTime = msec;
}

bool LinuxCM730::IsUpdateTimeout()
{
    if(GetUpdateTime() > m_UpdateWaitTime)
        return true;

    return false;
}

double LinuxCM730::GetUpdateTime()
{
    double time;

    time = GetCurrentTime() - m_UpdateStartTime;
    if(time < 0.0)
        m_UpdateStartTime = GetCurrentTime();

    return time;
}

void LinuxCM730::Sleep(double msec)
{
    double start_time = GetCurrentTime();
    double curr_time = start_time;

    do {
        usleep((start_time + msec) - curr_time);
        curr_time = GetCurrentTime();
    } while(curr_time - start_time < msec);
}
