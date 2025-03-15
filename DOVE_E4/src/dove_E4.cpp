#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <chrono>
#include "rs232/rs232.h"
#include "nmea_packet_protocol.h"
#include "nmea/parsers/gpgga.h"
#include "nmea/parsers/gprmc.h"
#include "nmea/parsers/gpchc.h"
#include <glog/logging.h>
#include "function_.hpp"
#include <iomanip>
#include <sys/msg.h>

using namespace std;
#define RADIANS_TO_DEGREES (180.0 / M_PI)
#define DEGREES_TO_RADIANS (M_PI / 180.0)
#define GPS_TIME_START_TIME_MSEC (315964800000)
#define MINUTES_IN_WEEK (10080)
#define _USE_MATH_DEFINES
#define MSGQUE_Nav2MAIN 500

#define DOVE_LINK
// #define DOVE_NOLINK

int main(int argc, char *argv[])
{
    NavMSG2Main message_Nav2main;
    initGlog(*argv);

#ifdef DOVE_LINK
    printf("\nYour NAV712 NMEA driver is currently running\nPress Ctrl-C to interrupt\n");
    std::string com_port = "/dev/ttyS1";
    int baud_rate = 115200, bytes_received;
    if (argc >= 3)
    {
        com_port = std::string(argv[1]);
        baud_rate = atoi(argv[2]);
    }
    nmea_decoder_t *nmea_decoder = (nmea_decoder_t *)malloc(sizeof(nmea_decoder_t));
    memset(nmea_decoder, 0, sizeof(nmea_decoder_t));
    nmea_record_t *nmea_record = (nmea_record_t *)malloc(sizeof(nmea_record_t));
    memset(nmea_record, 0, sizeof(nmea_record_t));
    nmea_init();
    char *values[255];
    if (OpenComport(const_cast<char *>(com_port.c_str()), baud_rate))
    {
        printf("Could not open serial port: %s \n", com_port.c_str());
        exit(EXIT_FAILURE);
    }
#endif

    // message que
    int msgQue_Nav2MAIN = -1;
    memset(&message_Nav2main, 0, sizeof(message_Nav2main));
    int Nav2GHB_BufSize = sizeof(message_Nav2main);
    msgQue_Nav2MAIN = msgget((key_t)MSGQUE_Nav2MAIN, 0666 | IPC_CREAT);
    while (msgQue_Nav2MAIN < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n", msgQue_Nav2MAIN);
        msgQue_Nav2MAIN = msgget((key_t)MSGQUE_Nav2MAIN, 0666 | IPC_CREAT);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 10ms=100Hz

#ifdef DOVE_LINK
        if ((bytes_received = PollComport(nmea_decoder_pointer(nmea_decoder), nmea_decoder_size(nmea_decoder))) > 0)
        {
            nmea_decoder_increment(nmea_decoder, bytes_received);
            // printf("Received bytes:%d\n", bytes_received);
            while (0 == nmea_record_decode(nmea_decoder, nmea_record))
            {
                std::string str((char *)nmea_record->data, nmea_record->length);
                // printf("Received string: %s", str.c_str());
                nmea_s *nmeaS = nmea_parse((char *)nmea_record->data, nmea_record->length, 1, values, sizeof(values) / sizeof(values[0]));
                if (!nmeaS)
                {
                    // printf("Parser NMEA sentence error\n");
                    continue;
                }
                switch (nmeaS->type)
                {
                case nmea_t::NMEA_GPRMC:
                {
                    static time_t now_t = time(NULL);
                    time_t diff_t = time(NULL) - now_t;
                    now_t = time(NULL);
                    if (diff_t == 0)
                        diff_t = 1;
                    nmea_gprmc_s *gprmc = (nmea_gprmc_s *)nmeaS;
                    double lat = gprmc->latitude.degrees + gprmc->latitude.minutes / 60.0;
                    double lon = gprmc->longitude.degrees + gprmc->longitude.minutes / 60.0;
                    printf("Rate2:%IdHz, Lat:%.8f, Lon:%.8f, Track:%.2f\n\n", 1 / diff_t, lat, lon, gprmc->track_deg);

                    message_Nav2main.mtype = 1;
                    // position
                    message_Nav2main.latitude = lat;
                    message_Nav2main.longitude = lon;
                    message_Nav2main.trackAngle = gprmc->track_deg;
#endif

#ifdef DOVE_NOLINK
                    message_Nav2main.mtype = 1;
                    message_Nav2main.latitude = 43.123456789;
                    message_Nav2main.longitude = 123.987654321;
                    message_Nav2main.trackAngle = 342.34566;
#endif
                    // save IMU_datas
                    LOG(INFO) << "longitude = " << setprecision(12) << message_Nav2main.longitude
                              << ", latitude = " << setprecision(12) << message_Nav2main.latitude
                              << ", heading = " << message_Nav2main.trackAngle << endl;
#ifdef DOVE_LINK

                    break;
                }
                default:
                {
                    // printf("Unknown packet: %s\n", nmea_record->header);
                    break;
                }
                }
                nmea_free(nmeaS);
            }
        }
#endif
        // send message
        int ret = msgsnd(msgQue_Nav2MAIN, (void *)&message_Nav2main, sizeof(message_Nav2main), IPC_NOWAIT);
        if (ret < 0)
        {
            printf("messge send failed,ret=%d\n", ret);
        }

    }                                // while (1)
    google::ShutdownGoogleLogging(); // 全局关闭glog
    return EXIT_SUCCESS;
} // end of main