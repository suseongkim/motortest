#ifndef MOTORTEST
#define MOTORTEST

#include <iostream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <mutex>
#include "rs232.h"

class MotorTest
{
	public:
		MotorTest();
		virtual ~MotorTest();
		bool initialize();
		void sendMotorCmd(std::vector<int>);

	private:
        int mMotorCmds[4];
        int mNumOfMotor;
        int mPortNum;
        int mBodeRate;
        std::mutex mTxDataMutex;

};

#endif
