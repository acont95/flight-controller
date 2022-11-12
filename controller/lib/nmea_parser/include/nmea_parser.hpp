#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uart_interface.hpp>

/************** GSV Message Structs **************/
struct GSVRepeatedBlock {
    char* svid;
    char* elv;
    char* az;
    char* cno;
};

struct GSVMsg {
    char* id;
    char numMsg;
    char msgNum;
    char* numSV;
    GSVRepeatedBlock gsvRepeated[4];
    char* signalId;
    char* cs;
};

/************** RMC Message Structs **************/


struct RMCMsg {
    char* id;
    char* time;
    char status;
    char* lat;
    char NS;
    char* lon;
    char EW;
    char* spd;
    char* cog;
    char* date;
    char* mv;
    char mvEW;
    char posMode;
    char nacStatus;
    char* cs;
};

/************** GSA Message Structs **************/
struct GSARepeatedBlock {
    char* svid;
};

struct GSAMsg {
    char* id;
    char opMode;
    char navMode;
    GSARepeatedBlock svidRepeated[12];
    char* PDOP;
    char* HDOP;
    char* VDOP;
    char* systemId;
    char* cs;
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
    char* id;
    char* lat;
    char NS;
    char* lon;
    char EW;
    char* time;
    char status;
    char posMode;
    char* cs;
};

/************** VTG Message Structs **************/

struct VTGMsg {
    char* id;
    char* cogt;
    char cogtUnit;
    char* cogm;
    char cogmUnit;
    char* sogn;
    char sognUnit;
    char* sogk;
    char sogkUnit;
    char posMode;
    char* cs;
};

/************** TXT Message Structs **************/

struct TXTMsg {
    char* id;
    char* numMsg;
    char* msgNum;
    char* msgType;
    char* text;
    char* cs;
};

struct Location {
    double lat;
    double lon;
    int64_t degrees;
    int64_t billionths;
};

class NMEAParser {
    public:
        NMEAParser();
        void encode(char c);
        Location getLocation();
        bool locationUpdated();
    private:
        char buf[255];
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
        bool locUpdated = false;

        double str2lat(char* s, char ns);
        double str2lon(char* s, char ew);
        int64_t str2deg(char* s);
        int64_t str2billionths(char* s);
};