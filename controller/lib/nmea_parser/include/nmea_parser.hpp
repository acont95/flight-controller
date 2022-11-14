#pragma once

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uart_interface.hpp>

/************** GSV Message Structs **************/
struct GSVRepeatedBlock {
    char svid[16];
    char elv[16];
    char az[16];
    char cno[16];
};

struct GSVMsg {
    char id[16];
    char numMsg;
    char msgNum;
    char numSV[16];
    GSVRepeatedBlock gsvRepeated[4];
    char signalId[16];
    char cs[16];
};

/************** RMC Message Structs **************/


struct RMCMsg {
    char id[16];
    char time[16];
    char status;
    char lat[16];
    char NS;
    char lon[16];
    char EW;
    char spd[16];
    char cog[16];
    char date[16];
    char mv[16];
    char mvEW;
    char posMode;
    char nacStatus;
    char cs[16];
};

/************** GSA Message Structs **************/
struct GSARepeatedBlock {
    char svid[16];
};

struct GSAMsg {
    char id[16];
    char opMode;
    char navMode;
    GSARepeatedBlock svidRepeated[12];
    char PDOP[16];
    char HDOP[16];
    char VDOP[16];
    char systemId[16];
    char cs[16];
};

/************** GGA Message Structs **************/

struct GGAMsg {
    char id[16];
    char time[16];
    char lat[16];
    char NS;
    char lon[16];
    char EW;
    char quality;
    char numSV[16];
    char HDOP[16];
    char alt[16];
    char altUnit;
    char sep[16];
    char sepUnit;
    char diffAge[16];
    char diffStation[16];
    char cs[16];
};

/************** GLL Message Structs **************/

struct GLLMsg {
    char id[16];
    char lat[16];
    char NS;
    char lon[16];
    char EW;
    char time[16];
    char status;
    char posMode;
    char cs[16];
};

/************** VTG Message Structs **************/

struct VTGMsg {
    char id[16];
    char cogt[16];
    char cogtUnit;
    char cogm[16];
    char cogmUnit;
    char sogn[16];
    char sognUnit;
    char sogk[16];
    char sogkUnit;
    char posMode;
    char cs[16];
};

/************** TXT Message Structs **************/

struct TXTMsg {
    char id[16];
    char numMsg[16];
    char msgNum[16];
    char msgType[16];
    char text[16];
    char cs[16];
};

struct Location {
    double lat;
    double lon;
};

class NMEAParser {
    public:
        NMEAParser();
        void encode(char c);
        Location getLocation();
        bool locationUpdated();
    private:
        char buf[128];
        int bufIdx = 0;
        char* fields[20];
        void parseMessage(char msg[], char** fields);

        void handleGSVMsg();
        void handleRMCMsg();
        void handleGSAMsg();
        void handleGGAMsg();
        void handleGLLMsg();
        void handleVTGMsg();
        void handleTXTMsg();

        GSVMsg gsvMessage;
        RMCMsg rmcMessage;
        GSAMsg gsaMessage;
        GGAMsg ggaMessage;
        GLLMsg gllMessage;
        VTGMsg vtgMessage;
        TXTMsg txtMessage;

        Location location;
        void setLocation();
        bool locUpdated = false;

        double str2lat(char* s, char ns);
        double str2lon(char* s, char ew);
        int64_t str2deg(char* s);
        int64_t str2billionths(char* s);
};