#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "cm730/CM740mp.h"
#include "cm730/dynamixel.h"

using namespace std;
using namespace cm730;
using namespace cm740;

void runMegapacket(double freq, struct timespec seconds)
{
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cm740test");
	ros::NodeHandle n;
	int connectError;
	CM740mp* board = new CM740mp("nimbro_op_interface/", "/nimbro_op_interface/");

	cout << "Testing behaviour of CM740 Megapacket mode" << endl << endl;
	cout << "Connect to CM740" << endl;
	connectError = board->connect();

	cout << "Resumed with Error: " << connectError << endl << endl;

	cout << "Ping to CM740" << endl;
	//int pingErr;
	int pingSucc;
	pingSucc = board->ping(200);

	cout << "Resumed with Err: " << pingSucc << endl << endl;

	cout << "Write on CM740 LED's" << endl;
	int writeErr1;
	writeErr1 = board->writeByte(CM730::ID_CM730, CM730::P_LED_PANEL, 0x03);
	cout << "Resumed with Error " << writeErr1 << endl << endl;

	cout << "Read baud rate from CM740" << endl;
	int readErr1, readValue = 0;
	readErr1 = board->readByte(CM730::ID_CM730, CM730::P_BAUD_RATE, &readValue);
	cout << "Resumed with Error: " << readErr1 << " and Value: " << readValue << endl << endl;

	sleep(1);

	MPBoard cmData;
	std::vector<MPData> servoData;
	board->addMegapacketServo(servoData,0x0C,0x6A);
	board->addMegapacketServo(servoData,0x0B,0x6A);

	cout << "Configuring megapacket mode for EX-106 servos with ID 11 and 12" << endl << endl;

	board->configureMegapacket(servoData,cmData);
	sleep(1);
	cout << "Enabling megapacket mode" << endl << endl;

	board->enableMegapacket(true);
	sleep(1);
	cout << "Turning on torque on servo ID 11 and 12" << endl << endl;

	servoData[0].torqueEnable = 1;
	servoData[1].torqueEnable = 1;
	servoData[0].torqueGoal = 0x03FF;
	servoData[1].torqueGoal = 0x03FF;
//	servoData[0].speedGoal = 0x00FF;
//	servoData[1].speedGoal = 0x00FF;
	servoData[0].f_writePosition = false;
	servoData[1].f_writePosition = false;

	board->writeMegapacket(servoData,cmData);
	sleep(1);
	cout << "Torque is now ON" << endl;
	cout << "Try to move any of the servos" << endl << endl;

	std::cout << "Press enter to continue ...";
	std::cin.get();

	cout << "Rotating servos to min position" << endl << endl;

	servoData[0].positionGoal = 0;
	servoData[1].positionGoal = 0;
	servoData[0].f_writePosition = true;
	servoData[1].f_writePosition = true;

	board->writeMegapacket(servoData,cmData);
	sleep(1);

	std::cout << "Press enter to continue ...";
	std::cin.get();

	cout << "Rotating servos to max position" << endl << endl;

	servoData[0].positionGoal = 0x0FFF;
	servoData[1].positionGoal = 0x0FFF;

	board->writeMegapacket(servoData,cmData);
	sleep(1);

	std::cout << "Press enter to continue ...";
	std::cin.get();

	cout << "Rotating servos back to min position" << endl << endl;

	servoData[0].positionGoal = 0;
	servoData[1].positionGoal = 0;

	board->writeMegapacket(servoData,cmData);
	sleep(1);

	std::cout << "Press enter to continue ...";
	std::cin.get();

	cout << "Turning off torque on servo ID 11 and 12" << endl << endl;

	servoData[0].torqueEnable = 0;
	servoData[1].torqueEnable = 0;
	servoData[0].f_writePosition = false;
	servoData[1].f_writePosition = false;

	board->writeMegapacket(servoData,cmData);
	sleep(1);
	cout << "Torque is now OFF, you can move the servos again" << endl << endl;

	std::cout << "Press enter to continue ...";
	std::cin.get();

	cout << "Trying to receive roughly 5 seconds of data" << endl << endl;
    for(int i=0;i<645;i++)
    {
    	board->receiveMegapacket(servoData,cmData);
    	cout << "-------------------------------------------------" << endl;
    	cout << "Board Voltage: " << (int)cmData.voltage << " AccX: " << (int)cmData.accX << " Health: " << (int)cmData.health << endl;
		cout << "Servo ID: " << (int)servoData[0].id << " position: " << (int)servoData[0].position << " packets received: " << (int)servoData[0].health << endl;
		cout << "Servo ID: " << (int)servoData[1].id << " position: " << (int)servoData[1].position << " packets received: " << (int)servoData[1].health << endl << endl;

    }

	cout << "Disabling megapacket mode" << endl << endl;

	board->enableMegapacket(false);
	sleep(1);

/*
	cout << "Write on CM740 LED's" << endl;
	int writeErr1;
	writeErr1 = board->writeByte(CM730::ID_CM730, CM730::P_LED_PANEL, 0x03);
	cout << "Resumed with Error " << writeErr1 << endl << endl;

	cout << "Read baud rate from CM730" << endl;
	int readErr1, readValue = 0;
	readErr1 = board->readByte(CM730::ID_CM730, CM730::P_BAUD_RATE, &readValue);
	cout << "Resumed with Error: " << readErr1 << " and Value: " << readValue << endl << endl;

	sleep(1);

	cout << "Write LED's via SyncWrite" << endl;
	int syncWriteErr;
	uint8_t param[2];
	param[0] = CM730::ID_CM730;
	param[1] = 0x02;
	syncWriteErr = board->syncWrite(CM730::P_LED_PANEL, 1, 1, param);
	cout << "Resumed with Error: " << syncWriteErr << endl << endl;

	board->setDynamixelPower(CM730::DYNPOW_ON);
	cout << endl << endl;

	sleep(1);

	std::vector<int> servos;
	servos.push_back(1);
	servos.push_back(2);
	board->updateTxBRPacket(servos);
	cout << "And now, try the BulkRead" << endl;
	int bulkReadErr;
	vector<BRData> brdata(2);
	BRBoard brboard;
	bulkReadErr = board->bulkRead(&brdata, &brboard);
	cout << "Resumed with Error: " << bulkReadErr << endl << endl;

	cout << "BulkRead Board Infos" << endl;
	cout << "Voltage: " << dec <<(int)brboard.voltage << endl;
	cout << "LED Pannel: " << (int)brboard.ledPanel << endl;
	cout << "AccelX: " << brboard.accX << endl;
	cout << "AccelY: " << brboard.accY << endl;
	cout << "AccelZ: " << brboard.accZ << endl << endl;

	cout << "------------------------------------------------------" << endl;
	cout << "Ping servo 1" << endl;
	pingErr =  board->ping(1);
	cout << "Resumed with Error: " << pingErr << endl << endl;

	cout << "Read Servo 1" << endl;
	int readErrS1, readValueS1;
	readErrS1 = board->readWord(1, DynamixelMX::P_PRESENT_POSITION_L, &readValueS1);
	cout << "Resumed with Error: " << readErrS1 << " and Value: " << dec << readValueS1 << endl << endl;

	board->writeByte(1, DynamixelMX::P_TORQUE_ENABLE, 1);



	if (pingSucc == 0 &&
	    writeErr1 == 0 &&
	    readErr1 == 0 &&
	    syncWriteErr == 0 &&
	    bulkReadErr == 0 &&
	    pingErr == 0)
		cout << "hooray!" << endl;
*/

	return 0;
}
