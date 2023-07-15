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

#include "apps/mode4App/Mode4Aircomp.h"
#include "common/LteControlInfo.h"
#include "stack/phy/packet/cbr_m.h"
#include "stack/mac/packet/Cr_m.h"
#include "inet/visualizer/base/QueueVisualizerBase.h"

Define_Module(Mode4Aircomp);

void Mode4Aircomp::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);
    if (stage==inet::INITSTAGE_LOCAL){
        // Register the node with the binder
        // Issue primarily is how do we set the link layer address

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();
        envi = ue->getParentModule();
        fullname_ = ue->getFullName();

        //Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);

        // Get mobility module of our UE
        ueMob = check_and_cast<inet::IMobility*>(ue->getSubmodule("mobility"));
        if (ueMob == nullptr) {
            EV << "Mode4Aircomp::initialize - stage " << stage
                      << "- Error: can't find UE[" << ue
                      << "] submodule 'mobility'" << endl;
        }

        // Get VeinsInetMobility module of our UE
        ueMobVeins = check_and_cast<veins::VeinsInetMobility*>(ue->getSubmodule("mobility"));
        if (ueMobVeins == nullptr) {
            EV << "Mode4Aircomp::initialize - stage " << stage
                    << "- Error: can't find UE[" << ue
                    << "] submodule 'VeinsInetMobility'" << endl;
        }
        ueMobCmd = ueMobVeins->getVehicleCommandInterface();
        if (ueMobCmd == nullptr) {
            EV << "Mode4Aircomp::initialize - stage " << stage
                    << "- Error: can't find UE[" << ue
                    << "] submodule 'TraCICommandInterface::Vehicle'" << endl;
        }

        EV << "Mode4Aircomp::initialize - stage "<< stage << " complete: "
                << "UeName[" << fullname_ << "], "
                << "nodeId[" << nodeId_ << "], "
                << "in envi[" << envi->getFullName() << "], "
                << endl;

    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {
        selfSender_ = NULL;
        nextSno_ = 0;
        currentPos = ueMob->getCurrentPosition();
        currentSpeed = ueMob->getCurrentSpeed();
        selfCAM = NULL;
        selfRisk = false;
        selfIAoISender_ = NULL;
        lastAction = 0;
        rcf = 1.1;
        lastIAoI = 0;
        lastCbr = 0;

        selfSender_ = new cMessage("selfSender");
        selfIAoISender_ = new cMessage("selfIAoISender");

        size_ = par("packetSize");
        period_ = par("period");
        periodMax_ = par("periodMax");
        periodMin_ = par("periodMin");
        priority_ = par("priority");
        duration_ = par("duration");
        periodOrigin = period_;

        sentMsg_ = registerSignal("sentMsg");
        delay_ = registerSignal("delay");
        rcvdMsg_ = registerSignal("rcvdMsg");
        cbr_ = registerSignal("cbr");
        period_signal = registerSignal("periodCAM");
        aircompAveSpeed_ = registerSignal("aircompAveSpeed");
        speed_ = registerSignal("speed");
        acceleration_ =  registerSignal("acceleration");
        aoi_ = registerSignal("aoi");
        selfTE_ = registerSignal("selfTE");
        iaoi_ = registerSignal("IAoI");
        aggIAoI_ = registerSignal("aggIAoI");
        action_ = registerSignal("action");
        deltaT_ = registerSignal("deltaT");
        nearUE_ = registerSignal("nearUE");
        minTTC_ = registerSignal("minTTC");
        maxDRAC_ = registerSignal("maxDRAC");
        leader_ = registerSignal("leader");

        cr_ = registerSignal("cr");

        double delay = 0.001 * intuniform(0, 1000, 0);
        scheduleAt((simTime() + delay).trunc(SIMTIME_MS), selfSender_);
        delay = 0.001 * intuniform(0, 1000, 0);
        scheduleAt((simTime() + delay).trunc(SIMTIME_MS), selfIAoISender_);

        EV << "Mode4Aircomp::initialize - stage "<< stage << " complete: "
                << "packetSize[" << size_ << "], "
                << "period[" << period_ << "], "
                << "priority[" << priority_ << "], "
                << "duration[" << duration_ << "]"
                << "ExternalID[" << ueMobVeins->getExternalId() << "]"
                << endl;

    }
}

void Mode4Aircomp::handleLowerMessage(cMessage* msg)
{
    EV << "Mode4Aircomp::handleMessage -Packet receive: " << msg << endl;
    if (msg->isName("CBR")) {
        EV << "Mode4Aircomp::handleMessage: CBR - [" << fullname_ << "], "
                << "curpos[" << ueMob->getCurrentPosition() << "]"
                << endl;
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        lastCbr = channel_load;
        emit(cbr_, channel_load);
        delete cbrPkt;

//        if(selfCAM!=NULL){
//            double aoiCAM = SIMTIME_DBL(simTime() - selfCAM->getTimestamp());
//            emit(aoi_, aoiCAM);
//        }
    } else if (msg->isName("CR")) {
        EV << "Mode4Aircomp::handleMessage: CR - [" << fullname_ << "], "
                << "curpos[" << ueMob->getCurrentPosition() << "]"
                << endl;
        Cr* crPkt = check_and_cast<Cr*>(msg);
        double currentCr = crPkt->getCr();
        lastCr = currentCr;
        emit(cr_, currentCr);
        delete crPkt;
    } else if (msg->isName("CAM")){
        Mode4Aircomp_Packet* pkt = check_and_cast<Mode4Aircomp_Packet*>(msg);
        if (pkt == 0)
            throw cRuntimeError("Mode4Aircomp::handleMessage - FATAL! Error when casting to Mode4Aircomp_Packet");

        // delete the previous CAM from the same Node
        for (unsigned int i = 0; i < nearCAM.size(); i++) {
            if (nearCAM[i]->getNodeId() == pkt->getNodeId()) {
                delete nearCAM[i];
                nearCAM.erase(nearCAM.begin()+i);
            }
        }

//        EV << "Mode4Aircomp::handleMessage - nearCAM[" << nearCAM.size() << "]: " << endl;
//        std::string fnTmp;
//        for (int i = 0; i < nearCAM.size(); i++) {
//            fnTmp = nearCAM[i]->getFullname();
//            EV << "SeqNo[" << nearCAM[i]->getSno() << "], "
//                    << "SenderNodeId[" << nearCAM[i]->getNodeId() << "], "
//                    << "Timestamp[" << pkt->getTimestamp() << "], "
//                    << "Period[" << period_ << "], "
//                    << "Pos[" << nearCAM[i]->getPos()<< "], "
//                    << "Speed[" << nearCAM[i]->getSpeed() << "], "
//                    << "Risk" << nearCAM[i]->getRisk() << "], "
//                    << "FullName[" << std::string(fnTmp) << "], "
//                    << "Delay[" << simTime() - nearCAM[i]->getTimestamp() << "]"
//                    << endl;
//        }

        // push received CAM into nearby vehicle CAM list
        nearCAM.push_back(pkt);

        // emit statistics
        simtime_t delay = simTime() - pkt->getTimestamp();
        emit(delay_, delay);
        emit(rcvdMsg_, (long)1);
        EV << "Mode4Aircomp::handleMessage - Received CAM Packet: "
                << "SeqNo[" << pkt->getSno() << "], "
                << "Timestamp[" << pkt->getTimestamp() << "], "
                << "SenderNodeId[" << pkt->getNodeId() << "], "
                << "Period[" << period_ << "], "
                << "Pos[" << pkt->getPos() << "], "
                << "Speed[" << pkt->getSpeed() << "], "
                << "Risk" << pkt->getRisk() << "], "
                << "FullName[" << pkt->getFullname() << "], "
                << "Delay[" << delay << "]"
                << "ExternalID[" << (ueMobVeins->getExternalId()).c_str() << "]"
                << endl;

    } else {
        EV << "Mode4Aircomp::handleMessage - Unrecognized Packet received: "<< msg << endl;
        delete msg;
    }
}

void Mode4Aircomp::handleSelfMessage(cMessage* msg)
{
    if (!strcmp(msg->getName(), "selfSender")){

        // Re-evaluate the CAM rate after CAM been sent
        IAoI_RateControl();

        // Get current mobility
        simtime_t currentTime = simTime();
        currentPos = ueMob->getCurrentPosition();
        currentSpeed = ueMob->getCurrentSpeed();

//        std::list<std::string> roadIdList = ueMobCmd->getPlannedRoadIds();
//        std::string roadId;
//        for(const auto &word : roadIdList){
//            roadId += word;
//        }

        // Replace method
        Mode4Aircomp_Packet* packet = new Mode4Aircomp_Packet("CAM");

        packet->setSno(nextSno_);
        packet->setTimestamp(currentTime);
        packet->setPeriod(period_);
        packet->setNodeId(nodeId_);
        packet->setPos(currentPos);
        packet->setSpeed(currentSpeed);
        packet->setRisk(selfRisk);
        packet->setFullname(fullname_.c_str());
        packet->setByteLength(size_);

        delete selfCAM;
        selfCAM = packet->dup();

//        duration_ = floor(SIMTIME_DBL((period_).trunc(SIMTIME_MS)));
//        duration_ = floor(period_.dbl() * 1000);
//        if(duration_ == 0){
//            EV << "Mode4Aircomp::handleSelfMessage - CAM Packet send: duration error" << endl;
//            duration_ = par("duration");
//        }

        auto lteControlInfo = new FlowControlInfoNonIp();

        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDirection(D2D_MULTI);
        lteControlInfo->setPriority(priority_);
        lteControlInfo->setDuration(duration_);
        lteControlInfo->setCreationTime(simTime());

        packet->setControlInfo(lteControlInfo);

        Mode4BaseApp::sendLowerPackets(packet);
        emit(sentMsg_, (long)1);
        emit(period_signal, SIMTIME_DBL(period_));



        EV << "Mode4Aircomp::handleSelfMessage - CAM Packet send: "
                << "SeqNo[" << nextSno_ << "], "
                << "Timestamp[" << currentTime << "], "
                << "NodeId[" << nodeId_ << "], "
                << "Period[" << period_ << "], "
                << "Pos[" << currentPos << "], "
                << "Speed[" << currentSpeed << "]"
                << "Risk[" << selfRisk << "], "
                << "FullName[" << std::string(fullname_) << "]"
                << "RoadID[" << ueMobCmd->getRoadId() << "]"
                << endl;



        // scehdule next CAM sender
        scheduleAt(simTime() + period_, selfSender_);

        nextSno_++;
    } else if (!strcmp(msg->getName(), "selfIAoISender")) {
        EV << "Mode4Aircomp::handleSelfMessage - selfIAoISender" << endl;
        IAoI_RateControl();
        scheduleAt(simTime() + 0.5, selfIAoISender_);
    }
    else
        throw cRuntimeError("Mode4Aircomp::handleMessage - Unrecognized self message");
}

double Mode4Aircomp::selfTrackError(){
    EV << "Mode4Aircomp::selfTrackError: start - " << selfRisk << endl;

    // selfCAM NULL check
    if (!selfCAM) {
        EV << "Mode4Aircomp::selfTrackError: selfCAM still NULL"<< endl;
        selfRisk = false;
        return 0;
    }

    // Get current mobility
    inet::Coord curPos = ueMob->getCurrentPosition();
    inet::Coord curSpeed = ueMob->getCurrentSpeed();

    // self-estimate location by selfCAM
    double t = SIMTIME_DBL(simTime() - selfCAM->getTimestamp());

    if (curPos == selfCAM->getPos()){
        emit(deltaT_, 0);
        EV << "Mode4Aircomp::selfTrackError: end - EQ - "
                << "delta-t[" << t << "], "
                << "curPos["<< curPos << "], "
                << "selfSpeed" << selfCAM->getSpeed() << "]"
                << endl;
        return 0;
    }
    else{
        // Line 1: Calculate the Self Tracking Error τp,v using (xv(t),yv(t)) and ( ̄xv(t),  ̄yv(t)) based on Eq .5.
        inet::Coord selfPos = selfCAM->getPos() + selfCAM->getSpeed() * t;
        double selfTrackError = curPos.distance(selfPos);
//        inet::Coord selfTrackErrorCoor = (curPos - selfCAM->getPos()) / t - selfCAM->getSpeed();
//        double selfTrackError = selfTrackErrorCoor.length();

        emit(deltaT_, t);

        EV << "Mode4Aircomp::selfTrackError: end - "
                << "delta-t[" << t << "], "
                << "curPos["<< curPos << "], "
                << "selfTE["<< selfTrackError << "], "
                << "selfPos" << selfCAM->getPos() << "], "
                << "selfSpeed" << selfCAM->getSpeed() << "]"
                << endl;

        // Line 8
        return selfTrackError;
    }


}

simtime_t Mode4Aircomp::IAoI_RateControl(){
    EV << "Mode4Aircomp::IAoI_RateControl: start - period[" << period_ << "]." << endl;
    double t = SIMTIME_DBL(simTime());
    // more detail in variable "lastAction"
    short act = 0;
    double optBP = 0;
    double cACF = 1;
    double cAoI = 1;
    double cTE = 1;
    double iaoi = 0;
    double cbr_desire = 0.6;

    double cCBR, cCR;



    // average the broadcast period
    simtime_t aveP = 0;
    for(unsigned int i = 0; i < nearCAM.size(); i++){
        // delete out of range CAM
        if (currentPos.distance(nearCAM[i]->getPos()) > 500){
            delete nearCAM[i];
            nearCAM.erase(nearCAM.begin()+i);
            continue;
        }
        aveP += nearCAM[i]->getPeriod();
    }

    // selfCAM NULL check
    if (!selfCAM) {
        EV << "Mode4Aircomp::IAoI_RateControl: selfCAM still NULL" << endl;
        return period_;
    }

    veins::VeinsInetMobility *nearUeMob;
    veins::TraCICommandInterface::Vehicle *ueMobVeinsCmd = ueMobVeins->getVehicleCommandInterface();
    double selfMinTTC = 0.0;
    double selfMaxDRAC = 0.0;
    ueMobVeinsCmd->getParameter("device.ssm.minTTC", selfMinTTC);
    ueMobVeinsCmd->getParameter("device.ssm.maxDRAC", selfMaxDRAC);
    emit(minTTC_, selfMinTTC);
    emit(maxDRAC_, selfMaxDRAC);
    double selfSpeed = 0.0;
    selfSpeed = ueMobVeinsCmd->getSpeed();
    if (selfSpeed > 0){
        std::pair<std::string, double> selfLeader = ueMobVeinsCmd->getLeader(3.0 * selfSpeed);
//        std::pair<std::string, double> selfLeader = ueMobVeinsCmd->getLeader(0);
        if(selfLeader.second != 0){
            emit(leader_, selfLeader.second/selfSpeed);
        }
    }
    else{
//        emit(leader_, 0);
    }

    // Line 1: calculate AoI
    //     different from TAoI
//    double aoiCAM = SIMTIME_DBL(simTime() - selfCAM->getTimestamp());
    double aoiCAM = 0;
    for (unsigned int i = 0; i < nearCAM.size(); i++) {
        double a = t - SIMTIME_DBL(nearCAM[i]->getTimestamp());
        if (a > 5){
            EV << "Mode4Aircomp::IAoI_RateControl: packet AoI too large" << a << endl;
            delete nearCAM[i];
            nearCAM.erase(nearCAM.begin()+i);
            continue;
        }
        else {
            aoiCAM += a;
        }
    }
    if (nearCAM.size() != 0){
        aoiCAM /= nearCAM.size();
    }
    else {
        aoiCAM = 0;
    }
    emit(aoi_, aoiCAM);

    // Line 1: calculate ACF(speed)
    double aveSpeed = aircompSpeed();
    emit(aircompAveSpeed_, aveSpeed);
    double curSpeed = ueMobCmd->getSpeed();
    emit(speed_, curSpeed);
    emit(acceleration_, ueMobCmd->getAcceleration());
    double acf = aveSpeed != 0 ? std::abs(aveSpeed - curSpeed) : 0;


    // Line 1: calculate selfTE
    double selfTE = selfTrackError();
    emit(selfTE_, selfTE);

    // Line 1: calculate IAoI
    // TODO: c-constant may change?
    cACF = 0.2;
    cAoI = 1;
    cTE = 10;
    iaoi = cAoI * aoiCAM + cACF * acf + cTE * selfTE;
    emit(iaoi_, iaoi);

    // Line2: calculate channel capacity and system constrain
    //                    15kHz * subcarrier * RB * subchannel
    //                                     [may change by cfg]
    //
    double channelCapa = 15 * 1000 *    12    * 16 *     1; // 2880000 (1),
    //                                  [change]
//    double channelCapa = 6500000; // MCS = 7
    double maxNumCAMCha = (channelCapa * cbr_desire) / size_; // numMsg/sec
    double maxNumCAMSys = 1 / periodMin_.dbl(); // 1/0.01 = 100
    double minNumCAMSys = 1 / periodMax_.dbl(); // 1/0.1 = 10

    // save IAoI and act for next round
    lastIAoI = iaoi;
    lastAction = act;
    emit(action_, act);

    // get aggIAoI
    double aggIAoI = aircompIAoI();
    aggIAoI += iaoi;
    emit(aggIAoI_, aggIAoI);

    cCBR = lastCbr < cbr_desire ? 1 : 1+lastCbr-cbr_desire;
    cCR = (lastCr/cbr_desire) < (iaoi/aggIAoI) ? 1 : 1 + (lastCr/cbr_desire) - (iaoi/aggIAoI);

    // Line : get Optimal broadcast period
    if(aggIAoI == iaoi){
        // no nearby vehicle, period not change
        optBP = period_.dbl();
    }
    else if(maxNumCAMCha <=  (numNearUe + 1) * maxNumCAMSys){
//        optBP = 1 / ( minNumCAMSys + (maxNumCAMCha - numNearUe) * (iaoi/aggIAoI) );
//        optBP = 1 / ( minNumCAMSys + (maxNumCAMCha -  (numNearUe + 1) * minNumCAMSys) * (iaoi/aggIAoI) );
        optBP = 1 / ( minNumCAMSys + (maxNumCAMCha -  (numNearUe + 1) * minNumCAMSys) * (iaoi/aggIAoI) * (1/(cCBR * cCR)));
    }
    else{
//        optBP = 1 / (minNumCAMSys + ( numNearUe * maxNumCAMSys - numNearUe) * (iaoi/aggIAoI) );
//        optBP = 1 / ( minNumCAMSys + (  (numNearUe+1) * maxNumCAMSys -  (numNearUe+1) * (1/periodMax_.dbl())) * (iaoi/aggIAoI) );
//        optBP = 1 / ( minNumCAMSys + numNearUe * (maxNumCAMSys - minNumCAMSys) * (iaoi/aggIAoI) );
        optBP = 1 / ( minNumCAMSys + numNearUe * (maxNumCAMSys - minNumCAMSys) * (iaoi/aggIAoI) * (1/(cCBR * cCR)) );
    }

    EV << "Mode4Aircomp::IAoI_RateControl: end - "
            << "period[" << period_ << "], "
            << "optBP[" << optBP << "], "
            << "IAoI[" << iaoi << "]."
            << endl;

    period_ = optBP;
    ueMobCmd->setAcionStepLength(optBP, true);
    emit(period_signal, SIMTIME_DBL(period_));

    return period_;
}

double Mode4Aircomp::aircompSpeed()
{
    cModule *nearUEModule;
    std::string nearUEname;
    double sumSpeed = 0;
    veins::VeinsInetMobility *nearUeMob;
    veins::TraCICommandInterface::Vehicle* nearUeMobCmd;

    numNearUe = 0;

    // get road id of current ue
    std::string ueRoadId = ueMobCmd->getRoadId();
    EV << "Mode4Aircomp::aircompSpeed: "
            << "Current UE is at road[" << ueRoadId <<"]"
            << endl;

    for (unsigned int i = 0; i < nearCAM.size(); i++){

        // get nearUE
        nearUEname = nearCAM[i]->getFullname();
        std::size_t lpos = nearUEname.find('[');
        if(lpos == std::string::npos){
            EV << "Mode4Aircomp::aircompSpeed: Error - "
                    << "nearUEname inappropriate [" << nearUEname << "]"
                    << endl;
            continue;
        }
        int index = std::stoi(nearUEname.substr( lpos + 1 , nearUEname.length() - lpos - 2));
        nearUEModule = envi->getSubmodule( nearUEname.substr( 0, lpos).c_str(), index);
        if(nearUEModule == nullptr){
            EV << "Mode4Aircomp::aircompSpeed: Error - "
                    << "Cannot find nearUE[" << nearUEname << "]"
                    << endl;
            continue;
        }

        // get nearUE mobility submodule
        nearUeMob = check_and_cast<veins::VeinsInetMobility*>(nearUEModule->getSubmodule("mobility"));
        if (nearUeMob == nullptr) {
            EV << "Mode4Aircomp::aircompSpeed: Error - "
                    << "can't find nearUEModule["<< nearUEname << "] "
                    << "submodule 'VeinsInetMobility'"
                    << endl;
            continue;
        }
        nearUeMobCmd = nearUeMob->getVehicleCommandInterface();
        std::string nearUeRoadId = nearUeMobCmd->getRoadId();
        if(ueRoadId.compare(nearUeRoadId) == 0) {
            numNearUe++;
            double nearUEspeed = nearUeMobCmd->getSpeed();
            sumSpeed += nearUEspeed;
            EV << "Mode4Aircomp::aircompSpeed: "
                    << "nearUEModule[" << nearUEname<< "], "
                    << "index[" << index << "], "
                    << nearUEModule << ", "
                    << "at same road[" << nearUeRoadId << "], "
                    << "speed[" << nearUEspeed << "]"
                    << endl;
        }
        else {
            EV << "Mode4Aircomp::aircompSpeed: "
                    << "nearUE[" << nearUEname<< "], "
                    << "index[" << index << "], "
                    << nearUEModule << ", "
                    << "at road[" << nearUeRoadId << "], "
                    << "Not at the same road"
                    << endl;
        }
    }

    // Submodule iterator
//    for (cModule::SubmoduleIterator i(envi); !i.end(); i++){
//            cModule *submodule = *i;
//            EV << "Mode4Aircomp::aircompSpeed: "
//                    << "envSubmodule[" << submodule->getFullName() << "]"
//                    << endl;
//    }

    emit(nearUE_, numNearUe);

    // Add self speed to sumSpeed
    sumSpeed += ueMobCmd->getSpeed();

    // Return average Speed
    if(numNearUe == 0){
        EV << "Mode4Aircomp::aircompSpeed: end - "
                << "No near UE."
                << endl;
        return sumSpeed;
    }
    else{
        EV << "Mode4Aircomp::aircompSpeed: end - "
                << "sumSpeed[" << sumSpeed << "], "
                << "numNearUE[" << numNearUe << "], "
                << "aveSpeed[" << sumSpeed / numNearUe << "]."
                << endl;

        return sumSpeed / (numNearUe + 1);
    }
}

double Mode4Aircomp::aircompIAoI()
{
    cModule *nearUE;
    std::string nearUEname;
    double sumIAoI = 0;
    veins::VeinsInetMobility *nearUeMob;
    veins::TraCICommandInterface::Vehicle* nearUeMobCmd;

    numNearUe = 0;

    // get road id of current ue
    std::string ueRoadId = ueMobCmd->getRoadId();
    EV << "Mode4Aircomp::aircompIAoI: "
            << "Current UE is at road[" << ueRoadId <<"]"
            << endl;

    for (unsigned int i = 0; i < nearCAM.size(); i++){
        // get nearUE
        nearUEname = nearCAM[i]->getFullname();
        std::size_t lpos = nearUEname.find('[');
        if(lpos == std::string::npos){
            EV << "Mode4Aircomp::aircompIAoI: Error - "
                    << "nearUEname inappropriate [" << nearUEname << "]"
                    << endl;
            continue;
        }
        int index = std::stoi(nearUEname.substr( lpos + 1 , nearUEname.length() - lpos - 2));
        nearUE = envi->getSubmodule( nearUEname.substr( 0, lpos).c_str(), index);
        if(nearUE == nullptr){
            EV << "Mode4Aircomp::aircompIAoI: Error - "
                    << "Cannot find nearUE[" << nearUEname << "]"
                    << endl;
            continue;
        }

        // get nearUE mobility submodule
        nearUeMob = check_and_cast<veins::VeinsInetMobility*>(nearUE->getSubmodule("mobility"));
        if (nearUeMob == nullptr) {
            EV << "Mode4Aircomp::aircompIAoI: Error - "
                    << "can't find nearUE["<< nearUEname << "] "
                    << "submodule 'VeinsInetMobility'"
                    << endl;
            continue;
        }
        nearUeMobCmd = nearUeMob->getVehicleCommandInterface();
        std::string nearUeRoadId = nearUeMobCmd->getRoadId();
        if(ueRoadId.compare(nearUeRoadId) == 0) {
            numNearUe++;

            Mode4Aircomp *nearUeAppl = check_and_cast<Mode4Aircomp*>(nearUE->getSubmodule("appl"));
            if (nearUeMob == nullptr) {
                EV << "Mode4Aircomp::aircompIAoI: Error - "
                          << "can't find nearUE[" << nearUEname << "] "
                          << "submodule 'appl'" << endl;
                continue;
            }

            double nearUEIAoI = nearUeAppl->getIAoI();
            sumIAoI += nearUEIAoI;
            EV << "Mode4Aircomp::aircompIAoI: "
                    << "nearUE[" << nearUEname<< "], "
                    << "index[" << index << "], "
                    << nearUE << ", "
                    << "at same road[" << nearUeRoadId << "], "
                    << "IAoI[" << nearUEIAoI << "]"
                    << endl;
        }
        else {
            EV << "Mode4Aircomp::aircompIAoI: "
                    << "nearUE[" << nearUEname<< "], "
                    << "index[" << index << "], "
                    << nearUE << ", "
                    << "at road[" << nearUeRoadId << "], "
                    << "Not at the same road"
                    << endl;
        }
    }

    // Submodule iterator
//    for (cModule::SubmoduleIterator i(envi); !i.end(); i++){
//            cModule *submodule = *i;
//            EV << "Mode4Aircomp::aircompSpeed: "
//                    << "envSubmodule[" << submodule->getFullName() << "]"
//                    << endl;
//    }

    emit(nearUE_, numNearUe);

    if(numNearUe == 0){
        EV << "Mode4Aircomp::aircompIAoI: end - "
                << "No near UE."
                << endl;
        return 0;
    }
    else{
        EV << "Mode4Aircomp::aircompIAoI: end - "
                << "sumIAoI[" << sumIAoI << "], "
                << "numNearUE[" << numNearUe << "], "
                << endl;

        return sumIAoI;
    }
}

double Mode4Aircomp::getIAoI(){
    return lastIAoI;
}

void Mode4Aircomp::finish()
{
    cancelAndDelete(selfSender_);
    cancelAndDelete(selfIAoISender_);
}

Mode4Aircomp::~Mode4Aircomp()
{
    binder_->unregisterNode(nodeId_);
}
