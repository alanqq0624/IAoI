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


package lte.apps.mode4App;
import lte.apps.mode4App.Mode4BaseApp;

simple Mode4Aircomp like Mode4BaseApp
{
    parameters:
        int packetSize = default(10); // Size of the actual packet itself
        int priority = default(3); // Priority of the packets sent
        int duration = default(1000); // MS before packet must be dropped
        double period @unit("s") = default(0.1s);
        double periodMax @unit("s") = default(0.1s);
        double periodMin @unit("s") = default(0.001s);
        int macNodeId = default(0);
        
        @signal[sentMsg];
        @statistic[sentMsg](title="Messages sent"; unit=""; source="sentMsg"; record=sum,vector);
        @signal[delay];
        @statistic[delay](title="Message Delay"; unit="s"; source="delay"; record=mean,vector);
        @signal[rcvdMsg];
        @statistic[rcvdMsg](title="Messages Received"; unit="s"; source="rcvdMsg"; record=sum,vector);
        @signal[cbr];
        @statistic[cbr](title="Channel Busy Ratio"; unit=""; source="cbr"; record=vector);
        @signal[periodCAM];
        @statistic[periodCAM](title="CAM broadcast period"; unit=""; source="periodCAM"; record=mean, vector);
        @signal[aircompAveSpeed];
        @statistic[aircompAveSpeed](title="Average speed via aircomp"; unit=""; source="aircompAveSpeed"; record=mean, vector);
        @signal[speed];
        @statistic[speed](title="Vehicle speed"; unit=""; source="speed"; record=mean, vector);
        @signal[acceleration];
        @statistic[acceleration](title="Vehicle acceleration"; unit=""; source="acceleration"; record=mean, vector);
        @signal[aoi];
        @statistic[aoi](title="Age of Information"; unit=""; source="aoi"; record=mean, vector);
        @signal[selfTE];
        @statistic[selfTE](title="self Track Error"; unit=""; source="selfTE"; record=mean, vector);
        @signal[IAoI];
        @statistic[IAoI](title="IAoI"; unit=""; source="IAoI"; record=mean, vector);
        @signal[aggIAoI];
        @statistic[aggIAoI](title="aggregated IAoI in the same road"; unit=""; source="aggIAoI"; record=mean, vector);
        @signal[action];
        @statistic[action](title="action taked in IAoI"; unit=""; source="action"; record=mean, vector);
        @signal[deltaT];
        @statistic[deltaT](title="delta T"; unit="s"; source="deltaT"; record=mean, vector);
        @signal[nearUE];
        @statistic[nearUE](title="Number of near UE"; unit=""; source="nearUE"; record=mean, vector);
        @signal[minTTC];
        @statistic[minTTC](title="minimum time to collision"; unit=""; source="minTTC"; record=mean, vector);
        @signal[maxDRAC];
        @statistic[maxDRAC](title="maximum DRAC"; unit=""; source="maxDRAC"; record=mean, vector);
        @signal[leader];
        @statistic[leader](title="Leader vehicle"; unit=""; source="leader"; record=mean, vector);
        
        @signal[cr];
        @statistic[cr](title="Channel Occupancy Ratio"; unit=""; source="cr"; record=vector);
        
        @display("i=block/source");
        
    gates:
        input lowerGateIn; // FROM NIC
        output lowerGateOut;  // TO NIC
}
