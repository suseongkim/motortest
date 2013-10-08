#include "motorTest.h"

using namespace std;

MotorTest::MotorTest()
{
    mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 0;
    mNumOfMotor = 4;
}

MotorTest::~MotorTest()
{
}

bool MotorTest::initialize(int portNum, int bodeRate)
{
    mPortNum = portNum;
    mBodeRate = bodeRate;

    if(RS232_OpenComport(mPortNum,mBodeRate))
    {
        cout << "cannot open motor serial port" << endl;
        return false;
    }

    return true;
}

void MotorTest::txData(std::vector<int> motorCmd)
{
    stringstream txDataStream;

    txDataStream << ":"

    for(int i=0; i<mNumOfMotor; i++)
    {
        txDataStream << motorCmd[i];

        if(i < mNumOfMotor - 1)
            txDataStream << ",";
        else
            txDataStream << "&";
    }

    txDataStream << "\n";
    string txData = txDataStream.str();
    char *txBuf = const_cast<char*>(txData.c_str());
    RS232_cputs(mPortNum,txBuf);
}

void MotorTest::sendMotorCmd(vector<int> motorCmd)
{
    txData(motorCmd);
}
