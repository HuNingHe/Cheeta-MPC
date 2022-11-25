#ifndef SHAREDMESSAGE_H
#define SHAREDMESSAGE_H
#include "SharedMemory.h"
#include "ControlParameters.h"
/*!
 * It should be noted that whenever a program attachs to this shared message, it must manually add one to "connected", 
 * otherwise the program will consider that the data has not been written. 
 * Each parameter name passed over cannot be empty, unnamed parameter will be neglected.
 * You should close the robot program before clicking Cancel in WebotsTool program
 */
struct SharedMessage{
    int connected = 0;                // This number represents the number of connection objects, and whenever an object is connected, the value should manually increment 1
	size_t nParameters = UINT64_MAX;  // parameters from tool to control robots in webots
	size_t nWebotsData = UINT64_MAX;  // parameters from webots to draw wave
    GamepadCommandShared gameCommand;		  // commands from Joystick to control robots in webots
	ControlParameter parameters[];	  // The first nParameters in the soft array are from Tool, The last nWebotsData are from webots
};
template class SharedMemoryObject<SharedMessage>;
#endif // !SHAREDMESSAGE_H
