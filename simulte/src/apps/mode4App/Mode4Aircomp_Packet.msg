//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//


//
// Packet reference from 
//     "/lte/src/apps/alert/AlertPacket.msg" and
//     "/veins/src/veins/modules/messages/DemoSafetyMessage.msg" 
//

cplusplus {{
	#include "inet/common/geometry/common/Coord.h"
	
	// copy from common/LteCommon.h
	#include "common/LteCommon.h"
//	typedef unsigned short MacNodeId;
}}

class noncobject inet::Coord;

packet Mode4Aircomp_Packet
{
    unsigned int sno;		// Sequence Number
    simtime_t timestamp;
    simtime_t period;
    unsigned short nodeId;
    inet::Coord pos;
    inet::Coord speed;
    bool risk;
    string fullname;
}
