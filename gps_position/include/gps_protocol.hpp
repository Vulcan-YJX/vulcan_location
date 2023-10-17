#ifndef GPS_PROTOCOL_HPP__
#define GPS_PROTOCOL_HPP__

#include <cstdint>

typedef struct
{         
    uint8_t   UTC_Time;      
    uint16_t  Latitude     : 10; 
    uint16_t  Latitude_Global :  6; 
    uint8_t   Longitude :  5; 
    uint8_t   Longitude_Global     :  1; 
    uint8_t   State    :  2;
    uint8_t   Satellite_num :  5; 
    uint8_t   HDOP     :  3; 
    uint8_t   Altitude;
    uint16_t  G_Altitude;         
    uint16_t  Diff_time;
    uint16_t  Diff_station; 
}__attribute__((packed)) ATK_1218_BD_t;


#endif /*GPS_PROTOCOL_HPP__*/
