//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

/**
 * Mode4App is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#ifndef _LTE_MODE4APP_H_
#define _LTE_MODE4APP_H_

#include <vector>
#include <cstring>

#include "apps/mode4App/Mode4BaseApp.h"
#include "corenetwork/binder/LteBinder.h"

#include "inet/mobility/contract/IMobility.h"
#include "apps/mode4App/Mode4Aircomp_Packet_m.h"

#include "inet/applications/base/ApplicationBase.h"
#include "inet/common/INETDefs.h"
#include "inet/mobility/base/MobilityBase.h"
#include "veins_inet/veins_inet.h"
#include "veins_inet/VeinsInetMobility.h"


class Mode4Aircomp : public Mode4BaseApp {

public:
    ~Mode4Aircomp() override;

protected:
    //sender
    int size_;
    int nextSno_;
    int priority_;
    int duration_;
    simtime_t period_;
    simtime_t periodMax_;
    simtime_t periodMin_;
    simtime_t periodOrigin;
    inet::Coord currentPos;
    inet::Coord currentSpeed;

    simsignal_t sentMsg_;
    simsignal_t delay_;
    simsignal_t rcvdMsg_;
    simsignal_t cbr_;
    simsignal_t period_signal;
    simsignal_t aircompAveSpeed_;
    simsignal_t speed_;
    simsignal_t acceleration_;
    simsignal_t aoi_;
    simsignal_t selfTE_;
    simsignal_t iaoi_;
    simsignal_t aggIAoI_;
    simsignal_t action_;
    simsignal_t deltaT_;
    simsignal_t nearUE_;
    simsignal_t minTTC_;
    simsignal_t maxDRAC_;
    simsignal_t leader_;
    simsignal_t cr_;

    cMessage *selfSender_;

    LteBinder* binder_;
    MacNodeId nodeId_;
    inet::IMobility *ueMob;
    cModule *envi;
    std::string fullname_;

    veins::VeinsInetMobility *ueMobVeins;
    veins::TraCICommandInterface::Vehicle* ueMobCmd;

    // nearby vehicle CAM list
    std::vector<Mode4Aircomp_Packet*> nearCAM;
    Mode4Aircomp_Packet* selfCAM;

    bool selfRisk;
    // last action of rate change
    // -1 = DECR, 0 = SAME, 1 = INCR
    short lastAction = 0;
    // broadcast interval change factor β
    double rcf = 1.1;
    // previuos IAoI
    double lastIAoI = 0;
    cMessage* selfIAoISender_;
    int numNearUe = 0;
    double lastCbr = 0;
    double lastCr = 0;

   int numInitStages() const { return inet::NUM_INIT_STAGES; }

   /**
    * Grabs NED parameters, initializes gates
    * and the TTI self message
    */
   void initialize(int stage);

   void handleLowerMessage(cMessage* msg);

   /**
    * Statistics recording
    */
   void finish();

   /**
    * Main loop of the Mac level, calls the scheduler
    * and every other function every TTI : must be reimplemented
    * by derivate classes
    */
   void handleSelfMessage(cMessage* msg);

   /**
    * sendLowerPackets() is used
    * to send packets to lower layer
    *
    * @param pkt Packet to send
    */
   void sendLowerPackets(cPacket* pkt);

   /**
    * selfTrackError() is used
    * to calculate self estimate track error
    *
    * @param selfLastCAM: The CAM last time UE broadcast, use to estimate location estimated by other UE
    */
   double selfTrackError();

   /**
    * aircompSpeed() is used
    * to simulate the aircomp process
    * do the aggregate process
    *
    */
   double aircompSpeed();

   /**
    * aircompIAoI() is used
    * to simulate the aircomp process
    * do the aggregate process
    *
    */
   double aircompIAoI();


   simtime_t IAoI_RateControl();

   /**
    * getIAoI() is used
    * to get IAoI last time calculate
    *
    */
    double getIAoI();

};

#endif
