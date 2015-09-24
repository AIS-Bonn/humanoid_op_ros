// Small tool to configure & debug dynamixel actuators
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dynalib/dxl_protocol.h>
#include <dynalib/impl/linux_serial.h>
#include <dynalib/impl/debug_io.h>

#include <dynalib/setup.h>
#include <dynalib/device_setup.h>
#include <dynalib/devices/generic.h>
#include <dynalib/device_registry.h>

#include <getopt.h>
#include <cstdio>
#include <fstream>

#include <readline/readline.h>
#include <readline/history.h>

#include <time.h>
#include <libio.h>
#include <unistd.h>
#include <stdlib.h>
#include <algorithm>

// Namespaces
using namespace dynalib;

struct Command
{
	const char* identifier;
	void (*func)(char* args);
	const char* description;
	const char* usage;
};

extern Command g_cmds[];

class DynaRobotInfo
{
public:
	struct IDInfo
	{
		int id;
		std::string abbrev;
		std::string name;
		IDInfo() : id(0), abbrev("?"), name("?") {}
	};
	bool valid;
	std::string fileName;
	std::string robotName;
	std::map<int,IDInfo> IDMap;
	DynaRobotInfo() : valid(false) {}
};

DynaRobotInfo g_dri;

dynalib::LinuxSerial serial;
dynalib::DebugIO debug;
dynalib::DXLProtocol<dynalib::LinuxSerial> proto(&serial);

dynalib::Device* g_device = 0;
dynalib::Generic g_genericDevice;
int g_id = 0;

bool Debug = false;

uint8_t g_buf[256];

// For hold command
uint8_t g_holdPGain = 16;
uint16_t g_holdTorqueLimit = 1023;

char* trim(char* str)
{
	if(!(*str))
		return str;

	while(isspace(*str))
		str++;

	char* end = &str[strlen(str)-1];
	while(isspace(*end))
	{
		*end = 0;
		end--;
	}

	return str;
}

void write_reg(Device* device, int id, Device::RegisterInfo info, unsigned int value, bool showText = true)
{
	if(!device)
	{
		fprintf(stderr, "No device set to write to.\n");
		return;
	}

	if(info.size == 0)
		return;

	// Start timing
	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	if(showText) printf("Writing register addr %d, size %d\n", info.addr, info.size);
	if(!proto.device(id).writeRaw(info.addr, (uint8_t*)&value, info.size))
	{
		fprintf(stderr, "Could not write value.\n");
		return;
	}

	// Stop timing
	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug && showText) printf("Register write took %.3fms!\n", dt);
}

bool read_device(Device* device, int id, uint8_t* buffer)
{
	if(!proto.device(id).readRaw(0, buffer, device->readSize())) return false;
	return true;
}

bool read_device()
{
	if(!g_device) return false;
	return read_device(g_device, g_id, g_buf);
}

unsigned int get_reg_value(const Device::RegisterInfo& info, bool& ok, const uint8_t* buf)
{
	unsigned int value = 0;
	ok = true;
	switch(info.size)
	{
		case 0:
			ok = false;
			break;
		case 1:
			value = buf[info.addr];
			break;
		case 2:
		{
			LEValue<2>* wrapper = (LEValue<2>*)&buf[info.addr];
			value = (*wrapper)();
			break;
		}
		default:
			ok = false;
			fprintf(stderr, "Unsupported register size %d\n", info.size);
			break;
	}
	return value;
}

unsigned int get_reg_value(const Device::RegisterInfo& info, bool& ok)
{
	return get_reg_value(info, ok, g_buf);
}

Command* findCommand(const char* id)
{
	for(Command* cmd = g_cmds; cmd->identifier; cmd++)
	{
		if(strcmp(cmd->identifier, id) == 0)
			return cmd;
	}
	return 0;
}

// Returns the ID of a servo based on its name (returns first match), returns 0 if invalid, -1 if not found
int IDForName(const std::string& name)
{
	if(!g_dri.valid) return -1;
	if(name.empty() || name.at(0) == '?') return -1;
	
	for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
	{
		if(it->second.abbrev == name || it->second.name == name)
		{
			if(it->first < 1 || it->first > 253)
				return 0;
			else
				return it->first;
		}
	}

	return -1;
}

const char* NameForID(int id)
{
	if(!g_dri.valid || id < 1 || id > 253) return "?";
	std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.find(id);
	if(it == g_dri.IDMap.end()) return "?";
	return it->second.name.c_str();
}

const char* AbbrevForID(int id)
{
	if(!g_dri.valid || id < 1 || id > 253) return "?";
	std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.find(id);
	if(it == g_dri.IDMap.end()) return "?";
	return it->second.abbrev.c_str();
}

// Returns an ID in the range 1-253, 0 if an ID was parsed but was out of range, and -1 if an ID couldn't be parsed.
// This function automatically increments ptr to the first non-whitespace character after the parsed ID.
// Both names and IDs are parsed by this function.
int parseIDInc(const char*& ptr) // str is assumed to be a valid null-terminated string
{
	while(isspace(*ptr) && (*ptr != '\0')) ptr++;
	if(*ptr == '\0') return -1;

	int IDTmp = 0;
	if(sscanf(ptr, "%d", &IDTmp) == 1)
	{
		while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
		while( isspace(*ptr) && (*ptr != '\0')) ptr++;
	}
	else
	{
		const char* strStart = ptr;
		while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
		std::string name = std::string(strStart, (std::size_t)(ptr-strStart));
		while(isspace(*ptr) && (*ptr != '\0')) ptr++;
		IDTmp = IDForName(name);
		if(IDTmp <= 0) return -1;
	}

	if(IDTmp < 1 || IDTmp > 253) return 0;
	return IDTmp;
}

// Same as parseIDInc(), but it doesn't attempt to modify the passed pointer
int parseID(const char* ptr)
{
	return parseIDInc(ptr);
}

void parseMultiIDWorker(const char* ptr, std::vector<int>& IDList)
{
	// Clear the ID list
	IDList.clear();
	
	// Error checking
	if(!ptr)
	{
		if(g_device)
			IDList.push_back(g_id);
		else
			fprintf(stderr, "Not available without device.\n");
		return;
	}
	
	// Check for the all specifier
	if(strncmp(ptr, "all", 3) == 0)
	{
		// We need the robot definition for this to work
		if(!g_dri.valid)
		{
			fprintf(stderr, "Robot definition is needed but not available.\n");
			return;
		}

		// Add every ID that we know about to our list
		for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
			IDList.push_back(it->second.id);
		
		// Don't process any further
		return;
	}
	
	// Try to parse the required IDs from the arguments
	const char* ptrb;
	int IDTmp;
	while(true)
	{
		if(*ptr == '\0') break;
		ptrb = ptr;
		IDTmp = parseIDInc(ptr);
		if(IDTmp > 0)
		{
			IDList.push_back(IDTmp);
		}
		else if(IDTmp == 0)
		{
			fprintf(stderr, "ID argument(s) must be in the range 1-253!\n");
			return;
		}
		else if(IDTmp <= -1)
		{
			printf("Could not parse IDs from '%s'\n", ptrb);
			return;
		}
	}
}

void parseMultiID(const char* ptr, std::vector<int>& IDList)
{
	parseMultiIDWorker(ptr, IDList);
	std::sort(IDList.begin(), IDList.end());
	IDList.erase(std::unique(IDList.begin(), IDList.end()), IDList.end());
	printf("Successfully parsed %u IDs.\n", (unsigned) IDList.size());
}

void cmd_exit(char*)
{
	exit(1);
}

void fprint_usage(FILE* fp, const char* id)
{
	Command* cmd = findCommand(id);
	if(cmd) fprintf(fp, "Usage: %s\n\n", cmd->usage);
}

void cmd_help(char* args)
{
	Command* cmd = 0;
	bool specific;

	if(!args)
		specific = false;
	else
		specific = ((cmd = findCommand(args)) != 0);

	if(specific)
	{
		printf("Command: %s => %s\n", cmd->identifier, cmd->description);
		printf("Usage: %s\n", cmd->usage);
	}
	else
	{
		printf("\nAvailable commands:\n");
		for(cmd = g_cmds; cmd->identifier; cmd++)
		{
			printf("  %-10s %s\n", cmd->identifier, cmd->description);
		}
		printf("For more information on a particular command use: help <command>\n");
	}
	printf("\n");
}

void cmd_scan(char* args)
{
	int max_id = 253;
	if(args && (sscanf(args, "%d", &max_id) != 1))
	{
		fprintf(stderr, "Invalid argument '%s'\n", args);
		fprint_usage(stderr, "scan");
		return;
	}
	else if((max_id < 0) || (max_id > 253))
	{
		fprintf(stderr, "Maximum ID argument out of 0-253 range '%d'\n", max_id);
		fprint_usage(stderr, "scan");
		return;
	}
	
	Generic::RegionFromTo<Generic::MODEL_NUMBER, Generic::FIRMWARE_VERSION> region;

	printf("Scanning ID's 0 to %d:\n\n", max_id);
	printf("  ID |   Model   | Firmware version\n");
	printf("===================================\n");

	fd_set fds;
	int num_found = 0;
	for(int i = 0; i <= max_id; i++)
	{
		printf("[%3d]                               \r", i);
		fflush(stdout);
		
		bool ok = false;
		for(int tries = 0; tries < 2; ++tries)
		{
			if(proto.device(i).readRegion(&region))
			{
				ok = true;
				break;
			}
		}

		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		timeval wait;
		wait.tv_sec = 0;
		wait.tv_usec = 100;
		if(select(STDIN_FILENO+1, &fds, 0, 0, &wait) == 1) break;

		if(!ok)
			continue;

		const char* model = DeviceRegistry::deviceName(region.MODEL_NUMBER());
		if(model)
			printf(" %3d | %-9s | 0x%02X\n", i, model, region.FIRMWARE_VERSION());
		else
			printf(" %3d | 0x%04X    | 0x%02X\n", i, region.MODEL_NUMBER(), region.FIRMWARE_VERSION());
		num_found++;
	}

	printf("        \n");
	printf("Found %d device(s)!\n\n", num_found);
}

void cmd_wait(char* args)
{
	unsigned int usec = 0;
	bool wait = false;

	if(!args)
	{
		usec = 500e3;
		wait = true;
	}
	else
	{
		int tmp = 0;
		char* ptr = args;
		while(isspace(*ptr) && (*ptr != '\0')) ptr++;
		if(*ptr == '\0')
		{
			usec = 500e3;
			wait = true;
		}
		else if(sscanf(ptr, "%d", &tmp) != 1)
		{
			printf("Could not parse an integer number of ms to wait for from '%s'", ptr);
			fprint_usage(stderr, "wait");
			return;
		}
		else
		{
			if(tmp < 0) tmp = 0;
			usec = (unsigned int) tmp * 1e3;
		}
	}

	if(usec <= 0)
		return;

	if(!wait)
	{
		printf("Waiting %u ms...\r", (unsigned int)(usec / 1e3));
		fflush(stdout);
	}

	unsigned int sec = usec / 1e6;
	usec -= sec * 1e6;

	if(sec > 0) sleep(sec);
	if(usec > 0) usleep(usec);

	if(wait)
	{
		printf("Waiting for enter key...\r");
		fflush(stdout);
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		select(STDIN_FILENO+1, &fds, 0, 0, 0);
		int c; while ((c = getchar()) != '\n' && c != EOF);
	}
	else
	{
		printf("                                 \r");
		fflush(stdout);
	}
}

void cmd_foreach(char* args)
{
	if(!args)
	{
		fprintf(stderr, "Invalid arguments.\n");
		fprint_usage(stderr, "foreach");
		return;
	}

	char* ptr = args;
	while(isspace(*ptr) && (*ptr != '\0')) ptr++;
	if(*ptr == '\0')
	{
		fprintf(stderr, "Invalid arguments.\n");
		fprint_usage(stderr, "foreach");
		return;
	}
	const char* strStart = ptr;
	while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
	std::string modelString = std::string(strStart, (std::size_t)(ptr-strStart));
	while(isspace(*ptr) && (*ptr != '\0')) ptr++;
	if(*ptr == '\0')
	{
		fprintf(stderr, "Command to execute for each '%s' is missing.\n", modelString.c_str());
		fprint_usage(stderr, "foreach");
		return;
	}

	if(modelString == "*") modelString.clear();

	char* cmd_name = ptr;
	while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
	if(*ptr != '\0') *ptr++ = '\0';
	char* cmd_args = ptr;
	cmd_args = trim(cmd_args);
	if(*cmd_args == '\0')
		cmd_args = 0;
	
	Command* cmd = findCommand(cmd_name);
	if(!cmd)
	{
		fprintf(stderr, "Unknown command '%s'\n", cmd_name);
		return;
	}

	Generic::RegionFromTo<Generic::MODEL_NUMBER, Generic::FIRMWARE_VERSION> region;

	Device* origdev = g_device;
	int origid = g_id;

	printf("\n");
	fd_set fds;
	int num_found = 0, num_exec = 0;
	for(int id = 0; id <= 253; id++)
	{
		printf("[%3d]                               \r", id);
		fflush(stdout);
		
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		timeval wait;
		wait.tv_sec = 0;
		wait.tv_usec = 1000;
		if(select(STDIN_FILENO+1, &fds, 0, 0, &wait) == 1) break;

		if(!proto.device(id).readRegion(&region))
			continue;

		const char* model = DeviceRegistry::deviceName(region.MODEL_NUMBER());
		if(!model)
			continue;
		num_found++;

		std::string readModelStr(model);
		if(readModelStr.length() >= modelString.length())
		{
			if(readModelStr.compare(0, modelString.length(), modelString) == 0)
			{
				printf("Device ID %d: %s\n", id, model);

				Device* dev = DeviceRegistry::device(region.MODEL_NUMBER());
				if(!dev) dev = &g_genericDevice;
				g_device = dev;
				g_id = id;

				cmd->func(cmd_args);

				printf("\n");
				num_exec++;
			}
		}
	}

	printf("        \r");
	printf("Executed command for %d of %d device(s)!\n\n", num_exec, num_found);

	g_device = origdev;
	g_id = origid;
}

Device* deviceForID(int id)
{
	// Start timing
	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	Generic::RegionFromTo<Generic::MODEL_NUMBER, Generic::FIRMWARE_VERSION> region;
	if(!proto.device(id).readRegion(&region))
	{
		fprintf(stderr, "Could not read from device %d, using generic model\n", id);
		return &g_genericDevice;
	}

	// Stop timing
	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug) printf("Read of device took %.3fms!\n", dt);

	Device* dev = DeviceRegistry::device(region.MODEL_NUMBER());
	if(!dev)
	{
		fprintf(stderr, "Unknown model number 0x%04X, using Generic model\n", region.MODEL_NUMBER());
		dev = &g_genericDevice;
	}

	return dev;
}

void cmd_dev(char* args)
{
	if(!args)
	{
		if(!g_device)
			printf("No device is currently selected.\n");
		else
			printf("Current device is ID %d = %s = %s.\n", g_id, g_device->name(), NameForID(g_id));
		return;
	}
	
	bool puremode = false;
	if(strncmp(args, "--pure ", 7) == 0)
	{
		puremode = true;
		args = trim(args + 7);
	}

	if(strcmp(args, "none") == 0)
	{
		if(g_device && !puremode)
			printf("Unselecting current device (ID %d = %s = %s).\n", g_id, g_device->name(), NameForID(g_id));
		g_device = 0;
		g_id = 0;
		return;
	}

	int id = parseID(args);
	if(id <= 0)
	{
		fprintf(stderr, "Could not parse ID from '%s'\n", args);
		fprint_usage(stderr, "dev");
		return;
	}

	g_device = deviceForID(id);
	g_id = id;

	if(g_dri.valid && !puremode) printf("Device changed to ID %d (%s).\n", g_id, NameForID(g_id));
}

void print_register_value(const Device::RegisterInfo& info, unsigned int value) // Note: The assumption here is that the register value is at most 16-bit
{
	if(info.flags & Device::RegisterInfo::FLAG_HEX)
	{
		if(info.size == 1)
			printf("  %4d | %-26s |   0x%02X\n", info.addr, info.name, (unsigned short) value);
		else
			printf("  %4d | %-26s | 0x%04X\n"  , info.addr, info.name, (unsigned short) value);
	}
	else if(info.flags & Device::RegisterInfo::FLAG_SIGNED)
		printf("  %4d | %-26s | %6hd\n"  , info.addr, info.name, (  signed short) value);
	else
		printf("  %4d | %-26s | %6hu\n"  , info.addr, info.name, (unsigned short) value);
}

void print_register_value_pure(const Device::RegisterInfo& info, unsigned int value) // Note: The assumption here is that the register value is at most 16-bit
{
	if(info.flags & Device::RegisterInfo::FLAG_HEX)
	{
		if(info.size == 1)
			printf("0x%02X\n", (unsigned short) value);
		else
			printf("0x%04X\n", (unsigned short) value);
	}
	else if(info.flags & Device::RegisterInfo::FLAG_SIGNED)
		printf("%hd\n", (  signed short) value);
	else
		printf("%hu\n", (unsigned short) value);
}

void print_register(const Device::RegisterInfo& info)
{
	bool ok = true;
	unsigned int value = get_reg_value(info, ok); // Note: This internally sets ok to true before doing anything!
	if(ok) print_register_value(info, value);
}

void cmd_dump(char* args)
{
	Device* dev = NULL;
	int id = 0;
	bool all = false;
	
	if(!args)
	{
		id = g_id;
		dev = g_device;
	}
	else
	{
		const char* ptr = args;
		if(strncmp(ptr, "all", 3) == 0)
		{
			char ch = *(ptr + 3);
			if(ch == '\0' || isspace(ch))
			{
				all = true;
				ptr += 3;
			}
		}

		while(isspace(*ptr) && (*ptr != '\0')) ptr++;
		if(*ptr != '\0')
		{
			const char* origptr = ptr;
			id = parseIDInc(ptr);
			if(id <= 0)
			{
				fprintf(stderr, "Could not parse ID from '%s'\n", origptr);
				fprint_usage(stderr, "dump");
				return;
			}
			dev = deviceForID(id);
			
			if(strncmp(ptr, "all", 3) == 0)
			{
				char ch = *(ptr + 3);
				if(ch == '\0' || isspace(ch))
					all = true;
			}
		}
		else
		{
			id = g_id;
			dev = g_device;
		}
	}
	
	if(!dev || id == 0)
	{
		fprintf(stderr, "No device is selected.\n");
		return;
	}

	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	if(!read_device(dev, id, g_buf))
	{
		fprintf(stderr, "Could not read from device.\n");
		return;
	}

	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug) printf("Register read took %.3fms!\n", dt);

	printf("\nRegister dump for %s device at ID %d (%s):\n\n", dev->name(), id, NameForID(id));

	printf("  Addr |            Name            |  Value  \n");
	printf(" =============================================\n");

	for(unsigned int i = 0; i < dev->registerCount(); i++)
	{
		Device::RegisterInfo info = dev->registerInfo(i);
		if(all || !(info.flags & Device::RegisterInfo::FLAG_HIDDEN))
			print_register(info);
	}

	printf("\n");
}

void cmd_read(char* args)
{
	// Make sure a device has been selected first
	if(!g_device)
	{
		fprintf(stderr, "Not available without device.\n");
		return;
	}

	// Declare variables
	const unsigned int MAX_BYTES = 255;
	uint8_t values[MAX_BYTES];
	uint8_t addr = 0, num_bytes = 0;
	unsigned int tmp = 0;
	size_t i;
	
	// Catch the --pure flag if it's there
	bool puremode = false;
	if(strncmp(args, "--pure ", 7) == 0)
	{
		puremode = true;
		args = trim(args + 7);
	}

	// Try to parse some arguments
	int num;
	char buf[100];
	if(!args || ((num = sscanf(args, "%99s %u", buf, &tmp)) < 1))
	{
		fprintf(stderr, "Invalid arguments.\n");
		fprint_usage(stderr, "read");
		return;
	}
	if(tmp > MAX_BYTES)
	{
		fprintf(stderr, "Can only read a maximum of %d bytes!\n", MAX_BYTES);
		return;
	}
	else
		num_bytes = (uint8_t) tmp;

	// Try to parse a name out of the address parameter
	for(i = 0; i < g_device->registerCount(); i++)
	{
		Device::RegisterInfo info = g_device->registerInfo(i);
		if(info.name == 0) continue;
		if(strcmp(info.name, buf) == 0)
		{
			addr = info.addr;
			if((num != 2) || (num_bytes == 0)) // If failed to parse number of bytes or parsed a 0...
				num_bytes = info.size;
			break;
		}
	}

	// If parsing a name was unsuccessful then try to parse an address
	if(i == g_device->registerCount())
	{
		if(sscanf(buf, "%u", &tmp) != 1)
		{
			fprintf(stderr, "Invalid arguments.\n");
			fprint_usage(stderr, "read");
			return;
		}
		else
		{
			if(tmp > 255)
			{
				fprintf(stderr, "Address out of range.\n");
				return;
			}
			addr = (uint8_t) tmp;
			for(i = 0; i < g_device->registerCount(); i++)
			{
				Device::RegisterInfo info = g_device->registerInfo(i);
				if(info.name == 0) continue;
				if(info.addr == addr)
				{
					if((num != 2) || (num_bytes == 0)) // If failed to parse number of bytes or parsed a 0...
						num_bytes = info.size;
					break;
				}
			}
		}
	}

	// If we still don't have a number of bytes then just default to 1
	if(num_bytes == 0) num_bytes = 1;

	// Error checking on number of bytes to read
	if(num_bytes > MAX_BYTES)
	{
		fprintf(stderr, "Can only read a maximum of %d bytes!\n", MAX_BYTES);
		return;
	}
	if((uint8_t) (addr + num_bytes - 1) < addr)
	{
		fprintf(stderr, "Address range is not allowed to loop around 255!\n");
		return;
	}

	// Start timing
	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	// Read from the device
	if(!proto.device(g_id).readRaw(addr, values, num_bytes))
	{
		fprintf(stderr, "Could not read from device!\n");
		return;
	}

	// Stop timing
	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug) printf("Register read took %.3fms!\n", dt);

	// Display the results of the read
	bool found;
	if(!puremode)
	{
		printf("\nRegister read for %s device at ID %d (%s) => Address %u for %u bytes:\n\n", g_device->name(), g_id, NameForID(g_id), addr, num_bytes);
		printf("  Addr |            Name            |  Value  \n");
		printf(" =============================================\n");
	}
	for(uint8_t j = 0; j <= num_bytes - 1; j++)
	{
		found = false;
		for(i = 0; i < g_device->registerCount(); i++)
		{
			Device::RegisterInfo info = g_device->registerInfo(i);
			if(info.name == 0) continue;
			if(info.addr == addr + j)
			{
				found = true;
				if(info.size >= 2)
				{
					unsigned int regvalue = ((unsigned int) values[j+1]) << 8 | values[j];
					if(puremode)
						print_register_value_pure(info, regvalue);
					else
						print_register_value(info, regvalue);
					j += info.size - 1;
				}
				else if(info.size == 1)
				{
					if(puremode)
						print_register_value_pure(info, values[j]);
					else
						print_register_value(info, values[j]);
				}
				break;
			}
		}
		if(!found && !puremode) printf("  %4d | ---                        | %6hu\n" , addr + j, (unsigned short) values[j]);
	}
	if(!puremode)
		printf("\n");
}

void cmd_reset(char* args)
{
	int id;
	if(!args)
	{
		if(!g_device)
		{
			printf("No device is currently selected.\n");
			fprint_usage(stderr, "reset");
			return;
		}
		else id = g_id;
	}
	else
	{
		id = parseID(args);
		if(id <= 0)
		{
			fprintf(stderr, "Invalid argument '%s'.\n", args);
			return;
		}
	}

	if(proto.device(id).reset())
		printf("Digital reset instruction successfully sent to ID %d (%s).\n", id, NameForID(id));
	else
		printf("An error occurred while sending the digital reset instruction to ID %d (%s).\n", id, NameForID(id));
}

void cmd_showdef(char* args)
{
	// Declare variables
	Device* dev = 0;
	unsigned int id = 0;

	// Work out which device to show the known definition of...
	if(!args)
	{
		// No args => Take current device
		if(!g_device)
		{
			fprintf(stderr, "Cannot show definition of current device as no device has been selected.\n");
			return;
		}
		dev = g_device;
	}
	else
	{
		// Args specifies the name of a known device?
		const DeviceRegistry::DeviceMap& map = DeviceRegistry::map();
		for(DeviceRegistry::DeviceMap::const_iterator it = map.begin(); it != map.end(); it++)
		{
			if(strcmp(args, it->second->name()) == 0)
			{
				dev = it->second;
				break;
			}
		}

		// Args specifies the id of a device on the bus
		if(sscanf(args, "%u", &id) == 1)
		{
			if((id > 0) && (id < 254))
			{
				dev = deviceForID(id);
				printf("Device at ID %u detected as type %s!\n", id, dev->name());
			}
		}
	}

	// If we haven't found a device by now then we can't complete the request
	if(!dev)
	{
		fprintf(stderr, "Device or definition of device not found.\n");
		printf("Known devices:\n");
		const DeviceRegistry::DeviceMap& map = DeviceRegistry::map();
		for(DeviceRegistry::DeviceMap::const_iterator it = map.begin(); it != map.end(); ++it)
			printf(" - %s (0x%04X)\n", it->second->name(), it->first);
		return;
	}

	// Display the register list
	printf("\nRegister list for %s device:\nH = Hidden, X = Hexadecimal, S = Signed, U = Unsigned, 8 = 8-bit, 16 = 16-bit\n\n", dev->name());
	printf("  Addr |            Name            |  Value  \n");
	printf(" =============================================\n");
	for(unsigned int i = 0; i < dev->registerCount(); i++)
	{
		Device::RegisterInfo info = dev->registerInfo(i);
		if(!info.name) continue;
		printf("  %4d | %-26s |", info.addr, info.name);
		printf("  %c%c%c%-2d  \n", (info.flags & Device::RegisterInfo::FLAG_HIDDEN ? 'H' : ' '), (info.flags & Device::RegisterInfo::FLAG_HEX ? 'X' : ' '), (info.flags & Device::RegisterInfo::FLAG_SIGNED ? 'S' : 'U'), (info.size >= 2 ? 16 : 8));
	}
	printf("\n");
}

void handle_set_write(char* args, bool isSet)
{
	if(!g_device)
	{
		fprintf(stderr, "Not available without device.\n");
		return;
	}

	unsigned int value;
	char buf[100];
	if(!args || (sscanf(args, "%99s %u", buf, &value) != 2))
	{
		fprintf(stderr, "Invalid arguments.\n");
		if(isSet) fprint_usage(stderr, "set");
		else fprint_usage(stderr, "write");
		return;
	}

	for(size_t i = 0; i < g_device->registerCount(); ++i)
	{
		Device::RegisterInfo info = g_device->registerInfo(i);

		if(info.name == 0)
			continue;

		if(strcmp(info.name, buf) == 0)
		{
			write_reg(g_device, g_id, info, value);
			return;
		}
	}

	uint8_t addr;
	if(sscanf(buf, "%hhu", &addr) == 1)
	{
		for(size_t i = 0; i < g_device->registerCount(); ++i)
		{
			Device::RegisterInfo info = g_device->registerInfo(i);
			if(info.name == 0) continue;
			if(info.addr == addr)
			{
				printf("Writing to register '%s'\n", info.name);
				write_reg(g_device, g_id, info, value);
				return;
			}
		}
	}

	fprintf(stderr, "Unknown register.\n");
}

void cmd_write(char* args)
{
	handle_set_write(args, false);
}

void cmd_set(char* args)
{
	handle_set_write(args, true);
}

void readID(int id, dynalib::Device* dev)
{
	// Error checking
	if(dev == 0)
	{
		printf("Null device passed to hold handler (for ID %d)!\n", id);
		return;
	}

	// Declare variables
	Device::RegisterInfo PRESENT_POSITION;

	// Find the registers of the required name
	for(unsigned int i = 0; i < dev->registerCount(); i++)
	{
		Device::RegisterInfo info = dev->registerInfo(i);
		if(info.name != 0)
		{
			if(strcmp(info.name, "PRESENT_POSITION") == 0)
			{
				PRESENT_POSITION = info;
				break;
			}
		}
	}

	// Stop here if we didn't find one of the required registers
	if(PRESENT_POSITION.name == 0) { printf("PRESENT_POSITION register not found in device definition: Can't read position!\n"); return; }

	// Stop here if the register sizes are strange
	if(PRESENT_POSITION.size == 0 || PRESENT_POSITION.size > 2) { printf("Unexpected size of PRESENT_POSITION register (%u bytes).\n", (unsigned) PRESENT_POSITION.size); return; }

	// Read the current device position
	uint8_t data[2] = {0, 0};
	if(!proto.device(id).readRaw(PRESENT_POSITION.addr, data, PRESENT_POSITION.size)) { fprintf(stderr, "Could not read PRESENT_POSITION from device!\n"); return; }
	unsigned int value = (((unsigned int) data[1] << 8) | data[0]);
	if(value < 0 || value > 4096) { printf("Unexpected value read from PRESENT_POSITION register (%u).\n", value); return; }

	// Inform the user of the read value
	if(g_dri.valid)
		printf("Device %d: %4u (%s)\n", id, value, NameForID(id));
	else
		printf("Device %d: %4u\n", id, value);
}

void cmd_readpos(char* args)
{
	// Declare variables
	dynalib::Device* dev;
	int IDTmp;

	// If no arguments were provided then just read the current ID
	if(!args)
	{
		// Make sure we have a currently selected device
		if(!g_device)
		{
			fprintf(stderr, "Not available without device.\n");
			return;
		}

		// Display header
		printf("Reading PRESENT_POSITION field(s):\n");

		// Release the currently selected device
		readID(g_id, g_device);

		// That's all folks!
		return;
	}

	// Display header
	printf("Reading PRESENT_POSITION field(s):\n");

	// Handle case that we wish to read all the servos we know about
	if(strncmp(args, "all", 3) == 0)
	{
		// We need the robot definition for this to work
		if(!g_dri.valid)
		{
			printf("Robot definition is needed but not available.\n");
			return;
		}

		// Read every ID that we know about
		for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
		{
			dev = deviceForID(it->second.id);
			if(strncmp(dev->name(), "MX", 2) == 0) // Note: Only attempt to read devices whose name start with "MX"
			{
				if(dev != &g_genericDevice)
					readID(it->second.id, dev);
				else
					printf("   Skipping the generic device of ID %d (%s).\n", it->second.id, it->second.name.c_str());
			}
		}

		// Finished processing the command
		return;
	}

	// Try to parse the required IDs from the arguments
	const char* ptr = args;
	const char* ptrb;
	std::vector<int> IDList;
	while(true)
	{
		if(*ptr == '\0') break;
		ptrb = ptr;
		IDTmp = parseIDInc(ptr);
		if(IDTmp > 0)
		{
			IDList.push_back(IDTmp);
		}
		else if(IDTmp == 0)
		{
			fprintf(stderr, "ID argument(s) must be in the range 1-253!\n");
			return;
		}
		else if(IDTmp <= -1)
		{
			printf("Could not parse IDs from '%s'\n", ptrb);
			return;
		}
	}
	printf("Successfully parsed %u IDs.\n", (unsigned) IDList.size());

	// Read all the IDs as required
	for(std::size_t i = 0; i < IDList.size(); i++)
	{
		dev = deviceForID(IDList[i]);
		if(dev != &g_genericDevice)
			readID(IDList[i], dev);
		else
			printf("   Skipping the generic device of ID %d (%s).\n", IDList[i], NameForID(IDList[i]));
	}
}

void holdID(int id, dynalib::Device* dev)
{
	// Error checking
	if(dev == 0)
	{
		printf("Null device passed to hold handler (for ID %d)!\n", id);
		return;
	}

	// Display header
	if(g_dri.valid)
		printf("Device %d (%s):\n", id, NameForID(id));
	else
		printf("Device %d:\n", id);

	// Declare variables
	Device::RegisterInfo PRESENT_POSITION;
	Device::RegisterInfo GOAL_POSITION;
	Device::RegisterInfo TORQUE_LIMIT;
	Device::RegisterInfo P_GAIN;

	// Find the registers of the required name
	for(unsigned int i = 0; i < dev->registerCount(); i++)
	{
		Device::RegisterInfo info = dev->registerInfo(i);
		if(info.name != 0)
		{
			if     (strcmp(info.name, "PRESENT_POSITION") == 0) PRESENT_POSITION = info;
			else if(strcmp(info.name, "GOAL_POSITION"   ) == 0) GOAL_POSITION = info;
			else if(strcmp(info.name, "TORQUE_LIMIT"    ) == 0) TORQUE_LIMIT = info;
			else if(strcmp(info.name, "P_GAIN"          ) == 0) P_GAIN = info;
		}
	}

	// Stop here if we didn't find one of the required registers
	if(PRESENT_POSITION.name == 0) { printf("PRESENT_POSITION register not found in device definition: Can't hold!\n"); return; }
	if(GOAL_POSITION.name == 0)    { printf("GOAL_POSITION register not found in device definition: Can't hold!\n"); return; }
	if(TORQUE_LIMIT.name == 0)     { printf("TORQUE_LIMIT register not found in device definition: Can't hold!\n"); return; }
	if(P_GAIN.name == 0)           { printf("P_GAIN register not found in device definition: Can't hold!\n"); return; }

	// Stop here if the register sizes are strange
	if(PRESENT_POSITION.size == 0 || PRESENT_POSITION.size > 2) { printf("Unexpected size of PRESENT_POSITION register (%u bytes).\n", (unsigned) PRESENT_POSITION.size); return; }
	if(GOAL_POSITION.size == 0    || GOAL_POSITION.size > 2   ) { printf("Unexpected size of GOAL_POSITION register (%u bytes).\n", (unsigned) GOAL_POSITION.size); return; }
	if(PRESENT_POSITION.size != GOAL_POSITION.size) { printf("Size of PRESENT_POSITION and GOAL_POSITION registers do not match (%u vs. %u).\n", (unsigned) PRESENT_POSITION.size, (unsigned) GOAL_POSITION.size); return; }

	// Read the current device position
	uint8_t data[2] = {0, 0};
	if(!proto.device(id).readRaw(PRESENT_POSITION.addr, data, PRESENT_POSITION.size)) { fprintf(stderr, "Could not read PRESENT_POSITION from device!\n"); return; }
	unsigned int value = (((unsigned int) data[1] << 8) | data[0]);
	if(value < 0 || value > 4096) { printf("Unexpected value read from PRESENT_POSITION register (%u).\n", value); return; }

	// Write to the P_GAIN register if required
	if(g_holdPGain > 0)
	{
		printf("   Writing P_GAIN = %u\n", (unsigned) g_holdPGain);
		write_reg(dev, id, P_GAIN, g_holdPGain, false);
	}

	// Write to the TORQUE_LIMIT register if required
	if(g_holdTorqueLimit > 0)
	{
		printf("   Writing TORQUE_LIMIT = %u\n", (unsigned) g_holdTorqueLimit);
		write_reg(dev, id, TORQUE_LIMIT, g_holdTorqueLimit, false);
	}

	// Write the read position as the goal position
	printf("   Writing GOAL_POSITION = %u\n", (unsigned) value);
	write_reg(dev, id, GOAL_POSITION, value, false);
}

void cmd_test(char* args)
{
	// Declare variables
	dynalib::Device* dev;
	std::vector<int> IDList;
	std::map<int, dynalib::Device*> devMap;

	// Parse the ID parameter list
	parseMultiID(args, IDList);
	printf("\n");

	// Hold all the IDs as required
	for(std::size_t i = 0; i < IDList.size(); i++)
	{
		dev = deviceForID(IDList[i]);
		if(strncmp(dev->name(), "MX", 2) == 0 && dev != &g_genericDevice)
		{
			printf("Found device ID %d (%s): %s\n", IDList[i], dev->name(), NameForID(IDList[i]));
			devMap[IDList[i]] = dev;
		}
		else
			printf("Not using device ID %d (%s): %s\n", IDList[i], dev->name(), NameForID(IDList[i]));
	}
	printf("\n");
	
	if(IDList.empty()) return;
	std::vector<int>::iterator it = IDList.begin();
	std::map<int, int> failMap;
	bool toggle = true;
	
	printf("Reading from devices and checking for failures...\n");
	
	fd_set fds;
	while(1)
	{
		if(it == IDList.end())
		{
			it = IDList.begin();
			if(toggle) printf(" *  ");
			else printf("  * ");
			toggle = !toggle;
			if(failMap.empty())
				printf("NO FAILS YET");
			else
			{
				printf("Failures:");
				for(std::map<int, int>::iterator itf = failMap.begin(); itf != failMap.end(); itf++)
					printf(" ID%d=%d", itf->first, itf->second);
			}
			printf("\r");
			fflush(stdout);
		}
		
		int id = *it;
		dev = devMap[id];
		
		Generic::RegionFromTo<Generic::MODEL_NUMBER, Generic::FIRMWARE_VERSION> region;
		if(!proto.device(id).readRegion(&region))
		{
			if(failMap.find(id) == failMap.end())
				failMap[id] = 1;
			else
				failMap[id]++;
		}
		
		it++;

		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);

		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 10*1000;

		int ret = select(STDIN_FILENO+1, &fds, 0, 0, &timeout);
		if(ret == 1)
			break;
	}
}

void cmd_hold(char* args)
{
	// Declare variables
	int PGain, TorqueLimit, ret, IDTmp;
	dynalib::Device* dev;
	bool noArgs = false;
	const char* ptr;
	const char* ptrb;

	// Error checking
	if(!args)
	{
		ptr = "";
		noArgs = true;
	}
	else
	{
		ptr = args;
	}

	// Initialise config variables
	PGain = (int) g_holdPGain;
	TorqueLimit = (int) g_holdTorqueLimit;

	// Handle case of config command
	if(strncmp(ptr, "config", 6) == 0)
	{
		// Check if that's all
		if(!isspace(ptr[6]))
		{
			printf("Current P_GAIN: %d\n", (int) g_holdPGain);
			printf("Current TORQUE_LIMIT: %d\n", (int) g_holdTorqueLimit);
			return;
		}

		// Try to parse the config parameters
		ret = sscanf(ptr+7, "%d %d", &PGain, &TorqueLimit);

		// React appropriately
		if(ret <= 0)
		{
			fprintf(stderr, "Failed to parse the required parameters\n");
			fprint_usage(stderr, "hold");
		}
		if(ret >= 1)
		{
			if(PGain < 0 || PGain > 64)
				fprintf(stderr, "P_GAIN argument out of 0-64 range '%d'\n", PGain);
			else
			{
				g_holdPGain = (uint8_t) PGain;
				printf("New hold P_GAIN: %d\n", PGain);
			}
		}
		if(ret >= 2)
		{
			if(TorqueLimit < 0 || TorqueLimit > 1023)
				fprintf(stderr, "TORQUE_LIMIT argument out of 0-1023 range '%d'\n", TorqueLimit);
			else
			{
				g_holdTorqueLimit = (uint16_t) TorqueLimit;
				printf("New hold TORQUE_LIMIT: %d\n", TorqueLimit);
			}
		}

		// Finished processing the command
		return;
	}

	// Handle case that we wish to hold all the servos we know about
	if(strncmp(ptr, "all", 3) == 0)
	{
		// We need the robot definition for this to work
		if(!g_dri.valid)
		{
			printf("Robot definition is needed but not available.\n");
			return;
		}

		// Hold every ID that we know about
		for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
		{
			dev = deviceForID(it->second.id);
			if(strncmp(dev->name(), "MX", 2) == 0) // Note: Only attempt to hold devices whose name start with "MX"
			{
				if(dev != &g_genericDevice)
					holdID(it->second.id, dev);
				else
					printf("   Skipping the generic device of ID %d (%s).\n", it->second.id, it->second.name.c_str());
			}
		}

		// Finished processing the command
		return;
	}

	// If no arguments were provided then just hold the current ID
	if(noArgs)
	{
		// Make sure we have a currently selected device
		if(!g_device)
		{
			fprintf(stderr, "Not available without device.\n");
			return;
		}

		// Hold the currently selected device
		holdID(g_id, g_device);

		// That's all folks!
		return;
	}

	// Try to parse the required IDs from the arguments
	std::vector<int> IDList;
	while(true)
	{
		if(*ptr == '\0') break;
		ptrb = ptr;
		IDTmp = parseIDInc(ptr);
		if(IDTmp > 0)
		{
			IDList.push_back(IDTmp);
		}
		else if(IDTmp == 0)
		{
			fprintf(stderr, "ID argument(s) must be in the range 1-253!\n");
			return;
		}
		else if(IDTmp <= -1)
		{
			printf("Could not parse IDs from '%s'\n", ptrb);
			return;
		}
	}
	printf("Successfully parsed %u IDs.\n", (unsigned) IDList.size());

	// Hold all the IDs as required
	for(std::size_t i = 0; i < IDList.size(); i++)
	{
		dev = deviceForID(IDList[i]);
		if(dev != &g_genericDevice)
			holdID(IDList[i], dev);
		else
			printf("   Skipping the generic device of ID %d (%s).\n", IDList[i], NameForID(IDList[i]));
	}
}

void releaseID(int id, dynalib::Device* dev)
{
	// Error checking
	if(dev == 0)
	{
		printf("Null device passed to hold handler (for ID %d)!\n", id);
		return;
	}

	// Display header
	if(g_dri.valid)
		printf("Device %d (%s):\n", id, NameForID(id));
	else
		printf("Device %d:\n", id);

	// Declare variables
	Device::RegisterInfo TORQUE_ENABLE;

	// Find the registers of the required name
	for(unsigned int i = 0; i < dev->registerCount(); i++)
	{
		Device::RegisterInfo info = dev->registerInfo(i);
		if(info.name != 0)
		{
			if(strcmp(info.name, "TORQUE_ENABLE") == 0)
			{
				TORQUE_ENABLE = info;
				break;
			}
		}
	}

	// Stop here if we didn't find the required register
	if(TORQUE_ENABLE.name == 0) { printf("TORQUE_ENABLE register not found in device definition: Can't release!\n"); return; }

	// Write to the TORQUE_ENABLE register
	printf("   Writing TORQUE_ENABLE = 0\n");
	write_reg(dev, id, TORQUE_ENABLE, 0, false);
}

void cmd_release(char* args)
{
	// Declare variables
	dynalib::Device* dev;
	int IDTmp;

	// If no arguments were provided then just release the current ID
	if(!args)
	{
		// Make sure we have a currently selected device
		if(!g_device)
		{
			fprintf(stderr, "Not available without device.\n");
			return;
		}

		// Release the currently selected device
		releaseID(g_id, g_device);

		// That's all folks!
		return;
	}

	// Handle case that we wish to release all the servos we know about
	if(strncmp(args, "all", 3) == 0)
	{
		// We need the robot definition for this to work
		if(!g_dri.valid)
		{
			printf("Robot definition is needed but not available.\n");
			return;
		}

		// Release every ID that we know about
		for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
		{
			dev = deviceForID(it->second.id);
			if(strncmp(dev->name(), "MX", 2) == 0) // Note: Only attempt to release devices whose name start with "MX"
			{
				if(dev != &g_genericDevice)
					releaseID(it->second.id, dev);
				else
					printf("   Skipping the generic device of ID %d (%s).\n", it->second.id, it->second.name.c_str());
			}
		}

		// Finished processing the command
		return;
	}

	// Try to parse the required IDs from the arguments
	const char* ptr = args;
	const char* ptrb;
	std::vector<int> IDList;
	while(true)
	{
		if(*ptr == '\0') break;
		ptrb = ptr;
		IDTmp = parseIDInc(ptr);
		if(IDTmp > 0)
		{
			IDList.push_back(IDTmp);
		}
		else if(IDTmp == 0)
		{
			fprintf(stderr, "ID argument(s) must be in the range 1-253!\n");
			return;
		}
		else if(IDTmp <= -1)
		{
			printf("Could not parse IDs from '%s'\n", ptrb);
			return;
		}
	}
	printf("Successfully parsed %u IDs.\n", (unsigned) IDList.size());

	// Release all the IDs as required
	for(std::size_t i = 0; i < IDList.size(); i++)
	{
		dev = deviceForID(IDList[i]);
		if(dev != &g_genericDevice)
			releaseID(IDList[i], dev);
		else
			printf("   Skipping the generic device of ID %d (%s).\n", IDList[i], NameForID(IDList[i]));
	}
}

void cmd_watch(char* args)
{
	if(!args)
	{
		fprintf(stderr, "No argument for 'watch' given. Abort.\n");
		fprint_usage(stderr, "watch");
		return;
	}
	char* cmd_args = strchr(args, ' ');
	char empty = '\0';

	if(cmd_args)
	{
		*cmd_args = '\0';
		cmd_args++;
	}
	else
		cmd_args = &empty;

	Command* cmd = findCommand(args);
	if(!cmd)
	{
		fprintf(stderr, "Unknown command '%s'\n", args);
		return;
	}

	fd_set fds;
	while(1)
	{
		cmd->func(cmd_args);

		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);

		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 300*1000;

		int ret = select(STDIN_FILENO+1, &fds, 0, 0, &timeout);
		if(ret == 1)
			break;
	}
}

void setup(char* args)
{
	if(!args)
	{
		fprintf(stderr, "No setup file given. Nothing to do.\n");
		fprintf(stderr, "Usage: setup PATH/TO/SETUP.yaml\n\n");
		return;
	}

	char path[100];
	if(sscanf(args, "%99s", path) != 1)
	{
		fprintf(stderr, "Could not parse filepath.\n");
		fprintf(stderr, "Usage: setup PATH/TO/SETUP.yaml\n");
		return;
	}

	Setup setup(path);

	if(not setup.parseFile())
	{
		fprintf(stderr, "Could not read file %s\n\n", path);
		return;
	}

	std::vector<DeviceSetup> setupList;
	int deviceIndex = 1;
	do
	{
		DeviceSetup set;
		bool success = false;
		try
		{
			success = setup.writeSetup(set);
		}
		catch (YAML::RepresentationException& e)
		{
			fprintf(stderr, "%s\n", e.what());
			return;
		}
		if (not success)
		{
			fprintf(stderr, "Could not match setup data for device %d\n\n", deviceIndex);
			return;
		}
		setupList.push_back(set);
		deviceIndex++;
	}
	while(setup.nextDevice());

	std::vector<DeviceSetup>::iterator it;
	for (it = setupList.begin(); it != setupList.end(); ++it)
	{
		int i;
		for (i = 0; i < it->registerCount(); ++i)
		{
			const DeviceSetup::Register* reg = it->reg(i);
			write_reg(it->device(), it->id(), reg->info, reg->value);
		}
	}

}

void cmd_listen(char* args)
{
	fd_set fds;
	int count = 0;
	while(1)
	{
		uint8_t c;
		uint16_t timeout = 1000; // 1ms timeout
		if(serial.readByte(&c, &timeout) == SUCCESS)
		{
			if(count == 0) printf("Got:");
			printf(" %02hhX", c);
			count++;
		}
		else
		{
			if(count != 0) printf("\n");
			count = 0;
		}

		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);

		timeval wait;
		wait.tv_sec = 0;
		wait.tv_usec = 1000;
		if(select(STDIN_FILENO+1, &fds, 0, 0, &wait) == 1) break;
	}
}

void cmd_ping(char* args)
{
	int id;
	if(!args)
	{
		if(!g_device)
		{
			printf("No device is currently selected.\n");
			fprint_usage(stderr, "ping");
			return;
		}
		else id = g_id;
	}
	else
	{
		id = parseID(args);
		if(id <= 0)
		{
			fprintf(stderr, "Invalid argument '%s'\n", args);
			return;
		}
	}

	// Start timing
	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	if(proto.device(id).ping())
		printf("Ping successful for ID %d (%s).\n", id, NameForID(id));
	else
		printf("ID %d (%s) did not answer.\n", id, NameForID(id));

	// Stop timing
	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug) printf("Ping took %.3fms!\n", dt);
}

void cmd_setraw(char* args)
{
	if(!args)
	{
		fprint_usage(stderr, "setraw");
		return;
	}

	if(!g_device)
	{
		fprintf(stderr, "Not available without a device.\n");
		return;
	}

	if(!g_device)
	{
		fprintf(stderr, "Not available without a device.\n");
		return;
	}

	int addr;
	int bytes;
	int value;
	if(sscanf(args, "%d %d %d", &addr, &bytes, &value) != 3)
	{
		fprint_usage(stderr, "setraw");
		return;
	}

	if(bytes > 4)
	{
		fprintf(stderr, "setraw can write up to 4 bytes.\n");
		return;
	}

	// Start timing
	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	proto.device(g_id).writeRaw(addr, (const uint8_t*)&value, bytes);

	// Stop timing
	struct timespec stop_time;
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	double dt = 1e3 * difftime(stop_time.tv_sec, start_time.tv_sec);
	dt += 1e-6 * ((double)(stop_time.tv_nsec - start_time.tv_nsec));
	if(Debug) printf("Register write took %.3fms!\n", dt);
}

void cmd_robot(char* args)
{
	if(!g_dri.valid)
	{
		printf("No valid robot definition is currently in use.\n");
		return;
	}

	printf("\n");
	printf("Robot name: %s\n", g_dri.robotName.c_str());
	printf("Robot definition file: '%s'\n", g_dri.fileName.c_str());

	if(g_dri.IDMap.size() == 0)
	{
		printf("No IDs are listed in the current robot definition.\n");
	}
	else
	{
		printf("Robot device ID map:\n");
		for(std::map<int,DynaRobotInfo::IDInfo>::iterator it = g_dri.IDMap.begin(); it != g_dri.IDMap.end(); it++)
			printf("ID %3d: %-3s ==> %s\n", it->second.id, it->second.abbrev.c_str(), it->second.name.c_str());
	}
	printf("\n");
}

int check(char* args)
{
	char stpPath[100];
	if (sscanf(args, "%s", stpPath) != 1)
	{
		fprintf(stderr, "Wrong number of arguments.\n");
		fprintf(stderr, "Usage: dynalib --check /path/to/setup.yaml");
		return 1;
	}
	Setup setup(stpPath);
	if (! setup.parseFile())
	{
		fprintf(stderr, "Could not read file %s\n\n", stpPath);
		return 1;
	}

	do
	{
		int id = setup.deviceId();
		std::string name = setup.deviceName();
		std::string type = setup.deviceType();

		if (id < 0)
		{
			fprintf(stderr, "Couldn't identify device: id missing\n");
			return 1;
		}

		if (! proto.device(id).ping())
			return 1;
	}
	while (setup.nextDevice());


	return 0;
}

void setRobot(char* args)
{
	g_dri.valid = false;
	g_dri.robotName.clear();

	g_dri.fileName = args;
	if(g_dri.fileName.rfind(".robot") == std::string::npos)
		g_dri.fileName.append(".robot");

	std::ifstream ifs;
	ifs.open(g_dri.fileName.c_str(), std::ifstream::in);

	const char* strStart;
	std::string line;

	while(ifs.good())
	{
		std::getline(ifs, line);
		if(line.find_first_not_of(' ') == std::string::npos) continue;
		const char* ptr = line.c_str();

		while(isspace(*ptr)) ptr++;
		
		if((strncmp(ptr, "name ", 5) == 0) || (strncmp(ptr, "NAME ", 5) == 0))
		{
			ptr += 5;
			while(isspace(*ptr)) ptr++;

			if(*ptr == '\0') continue;

			strStart = ptr;
			while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
			g_dri.robotName = std::string(strStart, (std::size_t)(ptr-strStart));
		}
		else if((strncmp(ptr, "ID ", 3) == 0) || (strncmp(ptr, "id ", 3) == 0))
		{
			DynaRobotInfo::IDInfo idi;
			int id;

			ptr += 3;
			while(isspace(*ptr)) ptr++;

			id = 0;
			if(sscanf(ptr, "%d", &id) != 1) continue;
			if(id < 1 || id > 253) continue;
			idi.id = id;

			while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
			while(isspace(*ptr)) ptr++;
			if(*ptr == '\0') continue;

			strStart = ptr;
			while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
			idi.abbrev = std::string(strStart, (std::size_t)(ptr-strStart));

			while(isspace(*ptr)) ptr++;
			if(*ptr != '\0')
			{
				strStart = ptr;
				while(!isspace(*ptr) && (*ptr != '\0')) ptr++;
				idi.name = std::string(strStart, (std::size_t)(ptr-strStart));
			}

			g_dri.IDMap[idi.id] = idi;
		}
	}

	ifs.close();

	g_dri.valid = !g_dri.robotName.empty();
	if(!g_dri.valid)
		printf("\nWarning: Could not parse valid robot definition from file '%s'! Does it even exist?\n", g_dri.fileName.c_str());
}

Command g_cmds[] = {
	{"dev"    , cmd_dev    , "Select the device with a given ID, or display the currently selected one", "dev [<ID>]"},
	{"dump"   , cmd_dump   , "Dump the values of all the registers of a device to screen", "dump [all] [<ID>] [all]"},
	{"exit"   , cmd_exit   , "Exit dynatool", "exit"},
	{"foreach", cmd_foreach, "Execute a specified command for each device of a certain model", "foreach <model> <dyna_command>\nThe dynamixel command is executed for all devices whose model string starts with the specified model string.\nUse '*' for the model string to execute a command for all devices."},
	{"help"   , cmd_help   , "Display this help message", "help <command>"},
	{"hold"   , cmd_hold   , "Request a servo to hold its current position", "hold config [<P_GAIN>] [<MAX_TORQUE>]\n       hold [<ID1> ... <IDn>]\n       hold all\nZero values for the config parameters mean that the respective registers are not touched.\nThe hold all feature only attempts to hold devices whose name start with 'MX'."},
	{"listen" , cmd_listen , "Just continually read from the serial port and see what arrives", "listen"},
	{"ls"     , cmd_dump   , "Alias for the dump command", "ls [all] [<ID>] [all]"},
	{"ping"   , cmd_ping   , "Send a ping packet and wait for an answer", "ping [<ID>]"},
	{"read"   , cmd_read   , "Read a register/multiple consecutive registers", "read <reg_name|reg_address> [<num_bytes>]"},
	{"readpos", cmd_readpos, "Read the current position of an MX series servo", "readpos [<ID>]"},
	{"release", cmd_release, "Request a servo to turn its torque off and allow free movement", "release [<ID1> ... <IDn>]\n       release all\nThe release all feature only attempts to release devices whose name start with 'MX'."},
	{"reset"  , cmd_reset  , "Digitally reset a device (be careful with servos, refer to 'help reset')", "reset [<ID>]\nCM730  => Just reboots the firmware\nServos => Resets (!) all the EEPROM registers (except the ID), including the baud rate register.\n       => To recover do 'dev <CM730>' then 'write BAUD_RATE 34', and restart dynatool with '--baud 57600'.\n       => Then go 'dev <id>', write the CW/CCW limits and torque max/limit, and finally 'write BAUD_RATE 1'.\n       => Reset the CM730 ('reset <CM730>') to return to 1Mbps comms and have the servo(s) reinitialised.\n       => You can now switch back to dynatool with the normal baudrate."},
	{"robot"  , cmd_robot  , "Show the current robot definition in use", "robot"},
	{"scan"   , cmd_scan   , "Scan bus at current baud rate for responsive devices", "scan [<max_id>]"},
	{"set"    , cmd_set    , "Set a register value (must be a known register)", "set <reg_name|reg_address> <value>"},
	{"setraw" , cmd_setraw , "Directly write to a device's register space by address and length", "setraw <address> <num_bytes> <value>"},
	{"showdef", cmd_showdef, "Show the definition of a known device", "showdef [<device_name|ID>]"},
	{"test"   , cmd_test   , "Test a given set of servos", "test [<ID1> ... <IDn>]\n       test all"},
	{"wait"   , cmd_wait   , "Wait for a given number of milliseconds", "wait [time_ms]\nIf the time parameter is omitted then this command waits for the enter key to be pressed."},
	{"watch"  , cmd_watch  , "Execute a specified command repeatedly", "watch <dyna_command>"},
	{"write"  , cmd_write  , "Write to a register (alias of 'set')", "write <reg_name|reg_address> <value>"},
	{0, 0, 0}
};

static void usage(FILE* stream)
{
	fprintf(stream,
		"Usage: dynatool [options]\n"
		"\n"
		"Options:\n"
		"  --device DEV    Open serial device DEV\n"
		"  --baud BAUD     Use baudrate BAUD\n"
		"  --debug         Enable communication debug logging\n"
		"  --setup FILE    Set default values for all devices specified in FILE\n"
		"  --check FILE    Verify the given setup FILE\n"
		"  --robot FILE    Use the given robot specification file to interpret IDs\n"
		"  --info          Display robot information at start\n"
		"  --exec          Suppress all fancy printing and just execute a particular command\n"
	);
}

option g_options[] = {
	{"device", required_argument, 0, 'd'},
	{"baud", required_argument, 0, 'b'},
	{"debug", no_argument, 0, 'D'},
	{"setup", required_argument, 0, 's'},
	{"check", required_argument, 0, 'c'},
	{"robot", required_argument, 0, 'r'},
	{"info", no_argument, 0, 'i'},
	{"exec", required_argument, 0, 'e'},
	{0, 0, 0, 0}
};

int main(int argc, char** argv)
{
	const char* device = "/dev/ttyUSB0";
	char* execcmd = NULL;
	unsigned int baudrate = 1000000;
	bool info = false;

	while(1)
	{
		int c = getopt_long(argc, argv, "h", g_options, 0);
		if(c == -1)
			break;

		switch(c)
		{
			case 'd':
				device = optarg;
				break;
			case 'b':
				baudrate = atoi(optarg);
				break;
			case 'D':
				serial.setDebugEnabled(true);
				Debug = true;
				break;
			case 'h':
				usage(stdout);
				return 0;
			case 's':
				setup(optarg);
				return 0;
			case 'c':
				return check(optarg);
			case 'r':
				setRobot(optarg);
				break;
			case 'i':
				info = true;
				break;
			case 'e':
				execcmd = (char*) malloc(sizeof(char) * (strlen(optarg)+1));
				if(!execcmd) return 1;
				strcpy(execcmd, optarg);
				break;
			default:
				usage(stderr);
				return 1;
		}
	}

	if(!execcmd)
	{
		printf("\n");

		printf("Communicating with device: %s\n", device);
		printf("Using baudrate: %dbps\n", baudrate);
		if(g_dri.valid) printf("Using robot definition file: '%s'\n", g_dri.fileName.c_str());
		if(Debug) printf("Debug mode enabled.\n");
		printf("\n");

		printf("Known devices:\n");
		const DeviceRegistry::DeviceMap& map = DeviceRegistry::map();
		for(DeviceRegistry::DeviceMap::const_iterator it = map.begin(); it != map.end(); ++it)
			printf(" - %s (0x%04X)\n", it->second->name(), it->first);
		printf("\n");
		
		if(info)
		{
			cmd_robot(NULL);
			printf("\n");
		}
	}

	if(!serial.init(device, baudrate))
	{
		fprintf(stderr, "Could not init serial connection\n");
		return 1;
	}

	if(!execcmd)
		printf("Dynatool command prompt. Type 'help' or 'help <command>' for usage information.\n\n");

	char head[20] = {0};
	while(1)
	{
		char* line = NULL;
		if(execcmd)
			line = execcmd;
		else
		{
			if(!g_device)
				sprintf(head, "dyna> ");
			else
				sprintf(head, "dyna %d> ", g_id);
			line = readline(head);
			if(!line)
				break;
		}

		char* nextptr = trim(line);
		char* ptr;
		if(strlen(nextptr) > 0) add_history(nextptr);
		while(true)
		{
			ptr = nextptr;
			if(!ptr) break;

			nextptr = strchr(ptr, ';');
			if(nextptr) *nextptr++ = '\0';
			
			char* trimmed = trim(ptr);
			if(trimmed[0] == '\0') continue;

			char* c = strchr(trimmed, ' ');
			char* args = 0;

			if(c)
			{
				*c = 0;
				args = c+1;
			}

			Command* cmd = findCommand(trimmed);

			if(args)
				args = trim(args);

			if(cmd)
				cmd->func(args);
			else
			{
				fprintf(stderr, "Unknown command '%s'\n", trimmed);
				cmd_help(0);
				break;
			}
		}

		if(execcmd)
			return 0;
		else
			free(line);
	}
	printf("\n");

	return 0;
}
