// IO implementation for linux serial devices
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dynalib/impl/linux_serial.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>

namespace dynalib
{

LinuxSerial::LinuxSerial()
 : m_fd(-1)
 , m_writeSize(0)
 , m_readSize(0)
 , m_readIdx(0)
 , m_debug(false)
{
}

LinuxSerial::~LinuxSerial()
{
	close(m_fd);
}

void LinuxSerial::setDebugEnabled(bool on)
{
	m_debug = on;
}

bool LinuxSerial::init(const char* device, unsigned int baud)
{
	if(m_fd > 0)
		close(m_fd);

	m_fd = open(device, O_RDWR);
	if(m_fd < 0)
	{
		perror("Could not open serial device");
		return false;
	}

	termios term;
	if(tcgetattr(m_fd, &term) != 0)
	{
		perror("Could not get terminal attributes");
		return false;
	}

	cfmakeraw(&term);
	term.c_cflag |= CLOCAL;
	speed_t speed;
	switch(baud)
	{
		case   57600: speed =   B57600; break;
		case  115200: speed =  B115200; break;
		case 1000000: speed = B1000000; break;
		default:
			fprintf(stderr, "Unknown baud rate %d\n", baud);
			return false;
	}
	m_baudFactor = 1000000 / baud;

	cfsetspeed(&term, speed);

	if(tcsetattr(m_fd, TCSANOW, &term) != 0)
	{
		perror("Could not set terminal attributes");
		return false;
	}

	return true;
}

ReturnCode LinuxSerial::startTransmit()
{
	if(::write(m_fd, m_writeBuf, m_writeSize) != m_writeSize)
	{
		perror("Could not write");
		return FAILURE;
	}

	if(m_debug)
		debug("TX+ :", m_writeBuf, m_writeSize);

	m_writeSize = 0;

	return SUCCESS;
}

bool LinuxSerial::write(const uint8_t* data, uint8_t size)
{
	tcflush(m_fd, TCIFLUSH);
	if(::write(m_fd, data, size) != size)
	{
		perror("Could not write");
		return false;
	}

	if(m_debug)
		debug("TX  :", data, size);

	return true;
}

ReturnCode LinuxSerial::readByte(uint8_t* dest, uint16_t* timeout)
{
	if(m_readIdx != m_readSize)
	{
		*dest = m_readBuf[m_readIdx];
		m_readIdx++;
		return SUCCESS;
	}

	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(m_fd, &fds);

	timeval tim;
	uint64_t usecs = *timeout * m_baudFactor;
	tim.tv_sec = (usecs / 1000000);
	tim.tv_usec = (usecs % 1000000);

	while(1)
	{
		int ret = select(m_fd+1, &fds, 0, 0, &tim);
		if(ret < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;

			perror("Could not select()");
			return FAILURE;
		}

		if(ret == 0)
		{
			*timeout = 0;
			return TIMEOUT;
		}

		break;
	}

	int ret = read(m_fd, m_readBuf, sizeof(m_readBuf));
	if(ret < 0)
	{
		perror("Could not read()");
		return FAILURE;
	}

	if(m_debug)
		debug("  RX:", m_readBuf, ret);

	*timeout = (tim.tv_usec + tim.tv_sec * 1000000L) / m_baudFactor;
	*dest = m_readBuf[0];
	m_readIdx = 1;
	m_readSize = ret;

	return SUCCESS;
}

void LinuxSerial::debug(const char* prefix, const uint8_t* data, uint8_t size)
{
	fputs(prefix, stdout);
	for(uint8_t i = 0; i < size; ++i)
	{
		printf(" %02X", data[i]);
	}
	fputc('\n', stdout);
}

}
