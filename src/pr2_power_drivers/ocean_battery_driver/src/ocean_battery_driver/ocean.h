#pragma once
/**
 */
#include <string>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <list>
#include <utility>
#include "pr2_msgs/BatteryServer2.h"

namespace willowgarage
{
  namespace ocean
  {
    class ocean
    {
    public:
      static const int NMEA_MAX = 120;
      static const int MAXTAGLEN = 8;
      static const int MAXCHANNELS = 16;

      static const int INPUT_BUF_SIZE = 128;
      static const int OUTPUT_BUF_SIZE = 128;
      static const unsigned int MAX_PACKET_LENGTH = 120;
      static const int BAD_PACKET = -1;
      static const int NO_PACKET = 0;
      static const int NMEA_PACKET = 1;

      struct regPair {
        const std::string name;
        const std::string unit;
        const unsigned address;
      };


      ocean (int id, int debug = 0);
      ~ocean ();

      int run ();
      void setDebugLevel (int);
      void initialize (const std::string &input_dev);
      void read_file (const std::string &input);

    private:
      long int convertStringBase16( const char* input );
      void flush (void);        //Flushes the serial buffer
      int get_speed (void);
      void set_speed (int speed);
      void report (int errlevel, const char *fmt, ...);
      char *gpsd_hexdump (void *binbuf, size_t binbuflen);
      void nextstate (unsigned char c);
      void packet_accept (int packet_type);
      void packet_discard ();
      void character_discard ();
      ssize_t packet_parse (size_t newdata);
      ssize_t packet_get ();
      void packet_reset ();
      int nmea_send (const char *fmt, ...);
      void nmea_add_checksum (char *sentence);
      time_t mkgmtime (register struct tm *t);
      unsigned int nmea_parse ();
      unsigned int processController (int count, char *field[]);
      unsigned int processSystem (int count, char *field[]);
      unsigned int processBattery (int count, char *field[]);
      int commTest();
      void resetOcean();
      int string_send (const char *fmt, ...);

    private:
      int inputDevice;
#if (FILE_LOGGING > 0)
      int outputFile;
#endif
      struct termios ttyset;
      int currentBaudRate;
      int currentBaudRateIndex;
      int currentPort;          //Port on GPS A=0, B=1, C=2
      char *idString;
      int debuglevel;
      int sentenceCount;
      int currentBattery;

      //Parsing stuff
      int packetState,          // Current statemachine location
        packetType;             // Type of packet just identified
      unsigned char inbuffer[INPUT_BUF_SIZE + 1];
      unsigned char outbuffer[OUTPUT_BUF_SIZE + 1];
      size_t inbuflen, outbuflen;
      unsigned char *inbufptr;
      unsigned long char_counter;
      char tag[MAXTAGLEN];
      int sentenceLength;
      int cycleComplete;
      int acknowledge;          // set this true when we receive a valid acnowledge

    public:                    //Expose this for easy access to results
      static const struct regPair regList[];
      static const unsigned regListLength;

      pr2_msgs::BatteryServer2 server;
    };
  }
}
