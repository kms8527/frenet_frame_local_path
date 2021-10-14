#ifndef __OBU_MSG_H__ 
#define __OBU_MSG_H__

#include <unistd.h>
#include <stdint.h>  

#define DEVICE_ID_SIZE 3 

const uint8_t cbnu_id[3] = {0x8C, 0xB1, 0x87}; // 충북대학교 고유 ID

struct obu_tcp_header_t{
    enum comm_type{
        ETH = 0x0001
    };
    enum target_type{ 
        T_OBU = 3,
        T_CLIENT = 4
    }; 
    enum device_type{
        D_OBU = 0xBD, 
        D_CLIENT = 0xCE
    };
    uint16_t packet_type;
    uint8_t current_sequence;
    uint16_t payload_size;
    uint8_t device_type;
    uint8_t device_id[DEVICE_ID_SIZE];

} __attribute__((packed));

////////////////////////////////////////////////////////////////////////////////////
struct access_permission
{
    double lat;
    double lon;
    float elev;
} __attribute__((packed));

struct access_permission_response
{
    unsigned char response;
    unsigned char errorCode;
} __attribute__((packed));

struct vehicle_location
{
    double lat;
    double lon;
    float elev;
    unsigned short heading;
    unsigned char speed;
} __attribute__((packed));

struct call_list
{
    unsigned char numberCall;
    unsigned char numberAvailableCall;
    unsigned char numberIrregular;
} __attribute__((packed));

struct call_data
{
    unsigned char id;
    unsigned char status;
    unsigned char includeMission;
    unsigned short point;
    unsigned short distance;
    unsigned char irregularId;
    double sLatitude;
    double sLongitude;
    double eLatitude;
    double eLongitude;
} __attribute__((packed));

struct irregularLoc
{
    unsigned char irregularId;
    double lat1;
    double lon1;
    double lat2;
    double lon2;
    double lat3;
    double lon3;
    double lat4;
    double lon4;
} __attribute__((packed));

struct call_request
{
    unsigned char requestCallID;
} __attribute__((packed));

struct call_response
{
    unsigned char matchingCallID;
    unsigned char responseStatus;
    unsigned char errorCode;
} __attribute__((packed));

struct start_point_arrive
{
    unsigned char matchingCallID;
    double lat;
    double lon;
    unsigned char speed;
} __attribute__((packed));

struct getting_on_confirm
{
    unsigned char matchingCallID;
    unsigned char confirmResult;
    unsigned char errorCode;
} __attribute__((packed));

struct end_point_arrive
{
    unsigned char matchingCallID;
    double lat;
    double lon;
    unsigned char speed;
} __attribute__((packed));

struct getting_off_confirm
{
    unsigned char matchingCallID;
    unsigned char confirmResult;
    unsigned char errorCode;
} __attribute__((packed));

struct mission_complete_result
{
    unsigned char matchingCallID;
    unsigned char confirmResult;
} __attribute__((packed));

struct mission_giving_up
{
    unsigned char matchingCallID;
} __attribute__((packed));

enum lte_packet_id
{
    ACCESS_RESTRICTION_NOTIFICATION = 0x133F, // 접속 제한 알림
    ACCESS_PERMISSION_REQUEST = 0x3133, // 자율차 장치 접속 요청 메세지
    ACCESS_PERMISSION_RESPONSE = 0x1334, // 자율차 장치 접속 응답 메세지
    VEHICLE_LOCATION_INFORMATION = 0x3135, // 위치 정보 메세지
    LOCATION_MESSAGE_ERROR = 0x1335, // 위치 정보 전송 오류 알림 메세지
    TAXI_CALL_LIST = 0x1136, // 콜 리스트 메세지
    CALL_REQUEST = 0x3137, // 콜 요청 메세지
    CALL_RESPONSE = 0x1338, // 콜 응답 메세지
    STARTING_POINT_ARRIVE = 0x3139, // 출발지 도착 완료 메세지
    GETTING_ON_CONFIRM = 0x133A, // 승차 완료 확인 메세지
    END_POINT_ARRIVE = 0x313B, // 목적지 도착 완료 메세지
    GETTING_OFF_CONFIRM = 0x133C, // 하차 완료 확인 메세지
    MISSION_COMPLETE_REQUEST = 0x133D, // 미션 완료 결과 메세지
    MISSION_GIVING_UP = 0x133E // 미션 포기 메세지
};
////////////////////////////////////////////////////////////////////////////////////
#endif
