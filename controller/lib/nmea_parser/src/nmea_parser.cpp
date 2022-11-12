#include <nmea_parser.hpp>

NMEAParser::NMEAParser() {

}

void NMEAParser::parseMessage(char msg[], char* f[]) {

    // set end of message to null char
    char* msg_end = strchr(msg, '\r');
    *msg_end = '\0';

    int i = 0;
    while (msg) {
        f[i] = msg;
        i++;
        msg = strchr(msg, ',');
        if (msg) {
            *msg = '\0';
            msg++;
        }
    }
}

void NMEAParser::encode(char c) {
    buf[bufIdx] = c;
    
    // switch (c)
    // {
    // case '\n':
    //     printf("\\n \n");
    //     break;
    // case '\r':
    //     printf("\\r \n");
    //     break;
    // default:
    //     printf("%c\n", c);
    //     break;
    // }

    if ((c == '\n') && (buf[bufIdx - 1] == '\r')) {
        parseMessage(buf, fields);
        const char* msg_type = fields[0] + 3;

        if (!strcmp(msg_type, "GSV")) {
            // handleGSVMsg();
        } else if (!strcmp(msg_type, "RMC")) {
            // handleRMCMsg();
        } else if (!strcmp(msg_type, "GSA")) {
            // handleGSAMsg();
        } else if (!strcmp(msg_type, "GGA")) {
            handleGGAMsg();
            locUpdated = true;
        } else if (!strcmp(msg_type, "GLL")) {
            // handleGLLMsg();
        } else if (!strcmp(msg_type, "VTG")) {
            // handleVTGMsg();
        } else if (!strcmp(msg_type, "TXT")) {
            // handleTXTMsg();
        }

        bufIdx = 0;
    } else {
        locUpdated = false;
        bufIdx++;
    }
}

void NMEAParser::handleGGAMsg() {
    GGAMsg msg;
    strcpy(msg.id, fields[0]);
    strcpy(msg.time, fields[1]);
    strcpy(msg.lat, fields[2]);
    msg.NS = *fields[3];
    strcpy(msg.lon, fields[4]);
    msg.EW = *fields[5];
    msg.quality = *fields[6];
    strcpy(msg.numSV, fields[7]);
    strcpy(msg.HDOP, fields[8]);
    strcpy(msg.alt, fields[9]);
    msg.altUnit = *fields[10];
    strcpy(msg.sep, fields[11]);
    msg.sepUnit = *fields[12];
    strcpy(msg.diffAge, fields[13]);
    strcpy(msg.diffStation, fields[14]);
    strcpy(msg.cs, fields[15]);

    // printf("ggamsg lat: %s\n", msg.lat);
    // printf("ggamsg lon: %s\n", msg.lon);
    // printf("fields lat: %s\n", fields[2]);
    // printf("fields lon: %s\n", fields[4]);

    ggaMessage = msg;
}

Location NMEAParser::getLocation() {
    location.lat = str2lat(ggaMessage.lat, ggaMessage.NS);
    location.lon = str2lon(ggaMessage.lon, ggaMessage.EW);
    return location;
}

double NMEAParser::str2lat(char* s, char ns) {
    char d[3];
    strncpy(d,s,2);
    d[2] = '\0';

    int64_t degrees = strtol(d, NULL, 10);
    int64_t min1 = strtol(s+2, NULL, 10) * 100000;
    int64_t min2 = strtol(s+5, NULL, 10);
    double frac_degrees = (min1 + min2) / 6000000.0;
    int8_t sign = ns=='N' ? 1 : -1;

    return sign*(degrees + frac_degrees);
}  

double NMEAParser::str2lon(char* s, char ew) {
    char d[4];
    strncpy(d,s,3);
    d[3] = '\0';
    int64_t degrees = strtol(d, NULL, 10);
    int64_t min1 = strtol(s+3, NULL, 10) * 100000;
    int64_t min2 = strtol(s+6, NULL, 10);
    double frac_degrees = (min1 + min2) / 6000000.0;
    int8_t sign = ew=='E' ? 1 : -1;

    return sign*(degrees + frac_degrees);
}  

bool NMEAParser::locationUpdated() {
    return locUpdated;
}