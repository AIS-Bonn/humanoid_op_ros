// Thread for reading data from the serial connection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef READ_THREAD_H
#define READ_THREAD_H

#include <pthread.h>
#include <stdint.h>

#include <boost/circular_buffer.hpp>

namespace io
{

class ReadThread
{
public:
	ReadThread();
	~ReadThread();

	void setFile(int fd);

	void run();

	// User interface
	int read(uint8_t* data, size_t size, struct timespec* abstime);
	void flush();

	static void* start(void* ptr);
private:
	int m_fd;

	uint8_t* m_buf;
	volatile size_t m_writeIdx;  // byte to be written
	volatile size_t m_readIdx;   // byte to be read in the next read()

	pthread_mutex_t m_mutex;
	pthread_cond_t m_cond;
};

}

#endif
