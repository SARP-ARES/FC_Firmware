#ifndef GPS_CMD
#define GPS_CMD



struct GPSCmd {
    const char* cmd;
    size_t len;
};

// ------------------- Baud Rate (PMTK251) -------------------
static constexpr GPSCmd CMD_BAUD_9600   { "$PMTK251,9600*17\r\n",   18 };
static constexpr GPSCmd CMD_BAUD_38400  { "$PMTK251,38400*27\r\n",  19 };
static constexpr GPSCmd CMD_BAUD_57600  { "$PMTK251,57600*2C\r\n",  18 };
static constexpr GPSCmd CMD_BAUD_115200 { "$PMTK251,115200*1F\r\n", 20 };

// ------------------- Update Rate (PMTK220) -------------------
static constexpr GPSCmd CMD_UPDATE_1HZ  { "$PMTK220,1000*1F\r\n",   18 };
static constexpr GPSCmd CMD_UPDATE_5HZ  { "$PMTK220,200*2C\r\n",    17 };
static constexpr GPSCmd CMD_UPDATE_10HZ { "$PMTK220,100*2F\r\n",    17 };

// ------------------- Fix Control (PMTK500) -------------------
// Controls position fix interval
static constexpr GPSCmd CMD_FIXCTL_1HZ  { "$PMTK500,1000,0,0,0.0,0.0*1A\r\n",  32 };
static constexpr GPSCmd CMD_FIXCTL_5HZ  { "$PMTK500,200,0,0,0.0,0.0*29\r\n",   31 };
static constexpr GPSCmd CMD_FIXCTL_10HZ { "$PMTK500,100,0,0,0.0,0.0*2A\r\n",   30 };

// ------------------- Antenna Status --------------------------
static constexpr GPSCmd CMD_ENABLE_ANTENNA_STATUS { "$CDCMD,33,1*7C\r\n", 16 };
static constexpr GPSCmd CMD_DISABLE_ANTENNA_STATUS { "$CDCMD,33,0*7D\r\n", 16 };



#endif // GPS_CMD