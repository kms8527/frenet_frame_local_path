#pragma once
#include <iostream>
#include <math.h>

#define LOBYTE 0
#define HIBYTE 1
class Can
{
protected:
    int id;
    unsigned char len;
    unsigned char data[8];
    void setData(const unsigned char *_data)
    {
        if (_data != nullptr)
            memcpy(&data, _data, sizeof(_data));
        else
        {
            for (int i = 0; i < 8; i++)
                data[i] = 0;
        }
    }
    void setBit(unsigned char &byte, int num, bool value)
    {
        unsigned char var = pow(2, num);
        if (value)
            byte = byte | var;
        else
            byte = byte & (~var);
    }

    void setHalfByte(unsigned char &byte, int loc, unsigned char target)
    {
        //loc = 0: low, 1: high
        if (loc)
            byte = byte | (target & 0xf0);
        else
            byte = byte | (target & 0x0f);
    }

    bool getBit(unsigned char byte, int num)
    {
        bool data;
        data = (byte >> num) & 0x01;
        return data;
    }

    unsigned char getHalfByte(unsigned char byte, int loc)
    {
        //loc = 0: low, 1: high
        unsigned char data;
        if (loc)
            data = (byte >> 4) & 0x0f;
        else
            data = byte & 0x0f;
        return data;
    }

    unsigned short getTwoByte(unsigned char byte_low, unsigned char byte_high)
    {
        unsigned short data;
        data = ((byte_high & 0xff) << 8) | (byte_low & 0xff);
        return data;
    }

public:
    Can(const int _id = 0, const unsigned char _len = 0, unsigned char *_data = nullptr) : id(_id),
                                                                                           len(_len)
    {
        setData(_data);
    }
    const unsigned char *getCanData() { return data; }

    void printCanData(const unsigned char *data) { fprintf(stderr, "DATA: %2x %2x %2x %2x %2x %2x %2x %2x\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]); }
};
class VehicleSetting : public Can
{
public:
    //can id: 0x156
    //Little-endian
    //Baudrate: 500kbps
    //Tx Rate: 10ms
    //Tx timeout: 1000ms
    unsigned char can_156[8];

    //Maunal/Atuo 모드 변환 요청
    bool eps_enable;

    //Ignore driver intervention
    //운전자 개입 무시
    //false: 핸들 조작하면 자율주행모드 꺼짐
    //true: 핸들 조작해도 자율주행모드 유지 -> 핸들은 손움직이면서 acc만 할 때 쓸 수 있다. can 값을 넣어주면 can값을 따라간다.
    //브레이크나 엑셀밟으면 둘다 자율주행모드 꺼짐
    bool override_ignore;

    //min: 10
    //max: 250
    //defalut: 150
    //It is not equal deg/s
    //스티어링 각속도
    //1.0 * value
    unsigned char eps_speed;

    //Manual/Auto 모드 변환 요청
    //automated speed moudlue
    bool acc_enable;

    //AEB 동작 요청(이거 true하면 풀브레이킹인가??)
    bool aeb_enable;

    //P: 0x8
    //R: 0x4
    //N: 0x2
    //D: 0x1
    unsigned char gear_change;

    //Right : 0x4
    //Left : 0x2
    //Hazard: 0x1
    //None: 0x0
    unsigned char turn_signal_enable;

    unsigned char alive_count;

public:
    VehicleSetting(bool _eps_enable = 0, bool _override_ignore = 0, unsigned char _eps_speed = 0, bool _acc_enable = 0, bool _aeb_enable = 0, unsigned char _gear_change = 0, unsigned char _turn_signal_enable = 0, unsigned char _alive_count = 0) : Can(0x156, 8),
                                                                                                                                                                                                                                                       eps_enable(_eps_enable),
                                                                                                                                                                                                                                                       override_ignore(_override_ignore),
                                                                                                                                                                                                                                                       eps_speed(_eps_speed),
                                                                                                                                                                                                                                                       acc_enable(_acc_enable),
                                                                                                                                                                                                                                                       aeb_enable(_aeb_enable),
                                                                                                                                                                                                                                                       gear_change(_gear_change),
                                                                                                                                                                                                                                                       turn_signal_enable(_turn_signal_enable),
                                                                                                                                                                                                                                                       alive_count(_alive_count)
    {
        convertCan();
    }
    ~VehicleSetting() {}

    void setVehicleSetting(bool _eps_enable, bool _override_ignore, unsigned char _eps_speed, bool _acc_enable, bool _aeb_enable, unsigned char _gear_change, unsigned char _turn_signal_enable, unsigned char _alive_count)
    {
        // VehicleSetting(_eps_enable, _override_ignore, _eps_speed, _acc_enable, _aeb_enable, _gear_change, _turn_signal_enable, _alive_count);
        this->eps_enable = _eps_enable;
        this->override_ignore = _override_ignore;
        this->eps_speed = _eps_speed;
        this->acc_enable = _acc_enable;
        this->aeb_enable = _aeb_enable;
        this->gear_change = _gear_change;
        this->turn_signal_enable = _turn_signal_enable;
        this->alive_count = _alive_count;
        convertCan();
    }
    void convertCan()
    {
        setBit(data[0], 0, eps_enable);
        setBit(data[0], 2, override_ignore);

        eps_speed = eps_speed < 10 ? 10 : eps_speed;
        eps_speed = eps_speed > 250 ? 250 : eps_speed;
        data[1] = eps_speed;

        setBit(data[2], 0, acc_enable);
        setBit(data[2], 6, aeb_enable);
        setHalfByte(data[5], HIBYTE, gear_change);
        setHalfByte(data[5], LOBYTE, turn_signal_enable);
        data[7] = alive_count;
    }
};
class VehicleControl : public Can
{
public:
    //can id: 0x157
    //Little-endian
    //Baudrate: 500kbps
    //Tx Rate: 10ms
    //Tx timeout: dependency on 0x156
    short steer_command;
    double acceleration_command;

    VehicleControl() : Can(0x157, 8),
                       steer_command(0),
                       acceleration_command(0)
    {
    }

    //value * 0.1
    //[-500,500](deg)
    void setSteerAngle(short target)
    {
        steer_command = target;
        target *= 10;
        short upper, lower;
        upper = ((target >> 8) & 0xff);
        lower = (target & 0xff);
        data[0] = lower;
        data[1] = upper;
    }

    //value * 0.01 - 10.23
    //[-3.0,1.5] (m/s2)
    //1.5m/s로 하면 0->50km/h 까지 약 10초.
    //-3.0m/s로 하면 50->0km/h 까지 약 7.5초.
    //감속은 한번에 -3이 아니라 점진적으로 증가해서 멈추는 데 시간이 오래걸린다.
    //aeb를 사용해야할까..? ㅠ
    void setAcceleration(double target_t)
    {
        acceleration_command = target_t;
        short target = target_t * 100 + 1023;
        unsigned short upper, lower;
        upper = ((target >> 8) & 0xff);
        lower = (target & 0xff);
        data[3] = lower;
        data[4] = upper;
    }
};

//Id: 0x710
//Baudrate: 500kbps
//Tx rate: 20ms
//Tx timeout: 1000ms
class EpsStat : public Can
{
public:
    //manual/auto 피드백
    bool enable;

    //can신호가 일정시간 입력되지 않는 경우 발생
    //에러 발생시 AEB동작
    bool user_can_err;

    //조향제어모듈 에러
    bool err;

    //차량정보 에러
    bool vehicle_can_err;

    //조향각센서 에러
    bool sas_err;

    //조향 override 무시 여부 피드백
    bool override_ignore_stat;

    //조향 override 무시여부
    //override 시 약 1초간 '1'표시
    bool override_stat;

    //제어보드 상택
    //0: 비정상, 1: 초기상태, 2: 정상동작상태
    unsigned char control_board_stat;

    //자율주행 제어모드
    //0: none, 1: ready, 2: all on, 3: acc on, 4: steer on, other: error
    unsigned char control_stat;

    //조향각 센서 값
    //0.1*value
    //[-500,500](deg)
    double steer_angle;

    //조향토크 센서 값
    //0.01*value - 2048
    //[-20.48, 20.47]
    double steer_drv_torque;

    //조향토크 센서 값 왜 두개인지 모르겟다.
    //0.1*value - 2048
    //[-204.8, 204.7]
    double steer_out_torque;

    //alive count 피드백
    short alive_count;

public:
    EpsStat() : Can(0x710, 8),
                enable(0),
                control_board_stat(0),
                user_can_err(0),
                err(0),
                vehicle_can_err(0),
                sas_err(0),
                control_stat(0),
                override_ignore_stat(0),
                override_stat(0),
                steer_angle(0.0),
                steer_drv_torque(0.0),
                steer_out_torque(0.0),
                alive_count(0)
    {
    }
    ~EpsStat() {}
    void setValue(const unsigned char *_data)
    {
        setData(_data);
        enable = getBit(data[0], 0);
        control_board_stat = getHalfByte(data[0], LOBYTE) >> 2;
        user_can_err = getBit(data[0], 4);
        err = getBit(data[0], 5);
        vehicle_can_err = getBit(data[0], 6);
        sas_err = getBit(data[0], 7);

        control_stat = getHalfByte(data[1], LOBYTE);
        override_ignore_stat = getBit(data[1], 4);
        override_stat = getBit(data[1], 5);
        steer_angle = static_cast<double>(static_cast<short>(getTwoByte(data[2], data[3]))) * 0.1;
        bool sign_bit = false;
        sign_bit = getBit(data[5], 3);
        //        steer_drv_torque = (sign_bit ? getTwoByte(data[4], data[5]) | 0xf000 :  (getTwoByte(data[4], data[5]) & 0x0fff) - 2048) * 0.01;
        steer_drv_torque = static_cast<double>(static_cast<short>((getTwoByte(data[4], data[5]) & 0x0fff) - 2048)) * 0.01;
        sign_bit = getBit(data[6], 7);
        //        steer_out_torque = ((sign_bit ? (getTwoByte(data[5], data[6]) >> 4) | 0xf000 : (getTwoByte(data[5], data[6]) >> 4) & 0x0fff) - 2048) * 0.1;
        alive_count = data[7];
    }
};
//Id: 0x711
//Baudrate: 500kbps
//Tx rate: 10ms
//Tx timeout: 1000ms
class AccStat : public Can
{
public:
    //manual/auto 피드백
    bool enable;

    //can신호가 일정시간 입력되지 않는 경우 발생
    //에러 발생시 AEB동작
    bool user_can_err;

    //차량 제어모듈 에러
    bool vehicle_err;

    //차량 가감속 제어부 에러
    bool err;

    //AEB 동작여부
    bool aeb_action;

    //좌측 방향 지시등
    bool turn_left_enable;

    //우측 방향 지시등
    bool turn_right_enable;
    //비상등
    bool harzard_enable;

    //기어 정보
    //P: 0x0, R: 0x7, N: 0x6, D: 0x5;
    unsigned char gear_select;

    //좌측 후측방 경보
    unsigned char bsd_left;

    //우측 후측방 경보
    unsigned char bsd_right;

    //제어보드 상택
    //0: 비정상, 1: 초기상태, 2: 정상동작상태
    unsigned char control_board_stat;
    //자율주행 제어모드
    //0: none, 1: ready, 2: all on, 3: acc on, 4: steer on, other: error
    //[0,10]
    unsigned char control_stat;

    //차량의 현재 속도
    //[0, 255] (km/h)
    unsigned char speed;

    //종방향 가속도 센서 값
    double long_accel;

    //alive_count 피드백
    unsigned char alive_count;

public:
    AccStat() : Can(0x711, 8),
                enable(0),
                user_can_err(0),
                vehicle_err(0),
                err(0),
                aeb_action(0),
                turn_left_enable(0),
                turn_right_enable(0),
                harzard_enable(0),
                gear_select(0),
                bsd_left(0),
                bsd_right(0),
                control_board_stat(0),
                control_stat(0),
                speed(0),
                long_accel(0.0),
                alive_count(0)
    {
    }
    ~AccStat() {}
    void setValue(const unsigned char *_data)
    {
        setData(_data);
        enable = getBit(data[0], 0);
        user_can_err = getBit(data[0], 4);
        vehicle_err = getBit(data[0], 6);
        err = getBit(data[0], 7);
        aeb_action = getBit(data[6], 3);
        turn_left_enable = getBit(data[6], 2);
        turn_right_enable = getBit(data[6], 0);
        ;
        harzard_enable = getBit(data[6], 1);
        ;
        gear_select = getHalfByte(data[6], HIBYTE);
        bsd_left = getHalfByte(data[3], HIBYTE) & 0x03;
        bsd_right = getHalfByte(data[3], LOBYTE) & 0x03;
        control_board_stat = getHalfByte(data[0], LOBYTE) >> 1;
        control_stat = getHalfByte(data[1], LOBYTE);
        speed = data[2];
        // bool sign_bit = false;
        // sign_bit = getBit(data[5],2);
        // long_accel = static_cast<double>(((sign_bit ? getTwoByte(data[4], data[5]) | 0xf800 : getTwoByte(data[4], data[5]) & 0x07ff) - 1023) * 0.01);
        long_accel = static_cast<double>((static_cast<unsigned short>(getTwoByte(data[4], data[5]) & 0x07ff) - 1023)) * 0.01;
        alive_count = data[7];
    }
};

//Id: 0x712
//Baudrate: 500kbps
//Tx rate: 10ms
//Tx timeout: None
class WheelSpeed : public Can
{
public:
    //휠 속도
    //0.03125*value
    //[0, 511.96875]
    double front_left,
        front_right,
        rear_left,
        rear_right;

public:
    WheelSpeed() : Can(0x712, 8),
                   front_left(0.0),
                   front_right(0.0),
                   rear_left(0.0),
                   rear_right(0.0)
    {
    }
    ~WheelSpeed() {}
    void setValue(const unsigned char *_data)
    {
        setData(_data);
        front_left = getTwoByte(data[0], data[1]) * 0.03125;
        front_right = getTwoByte(data[2], data[3]) * 0.03125;
        rear_left = getTwoByte(data[4], data[5]) * 0.03125;
        rear_right = getTwoByte(data[6], data[7]) * 0.03125;
    }
};

//Id: 0x713
//Baudrate: 500kbps
//Tx rate: 10ms
//Tx timeout: None
class YawAndBrake : public Can
{
public:
    //횡방향 가속도 센서 값
    //0.01*(value -1023)
    //[-10.23, 10.23]
    double lat_accel;

    //요레이트 센서 값
    //0.01*(value - 4095)
    //[-40.95, 40.94]
    double yaw_rate;

    //브레이크 마스터실린더 압력 값
    //0.1*value
    //[0, 409.4]
    double brake_pressure;

public:
    YawAndBrake() : Can(0x713, 8),
                    lat_accel(0.0),
                    yaw_rate(0.0),
                    brake_pressure(0.0)
    {
    }
    ~YawAndBrake() {}
    void setValue(const unsigned char *_data)
    {
        setData(_data);
        lat_accel = static_cast<double>(static_cast<short>(getTwoByte(data[0], data[1]) - 1023)) * 0.01;
        yaw_rate = static_cast<double>(static_cast<short>(getTwoByte(data[3], data[4]) - 4095)) * 0.01;
        brake_pressure = static_cast<double>(getTwoByte(data[6], data[7])) * 0.1;
    }
};

//Id: 0x714
//Baudrate: 500kbps
//Tx rate: 10ms
//Tx timeout: None
class RadarStat : public Can
{
public:
    //[0,4]
    unsigned char radar_object_state;

    //0.1*(value -20)
    //[-20, 31.1]
    double radar_object_lateral_pos;

    //0.1*value
    //[0, 204.7]
    double radar_object_distant;

    //0.1*(value - 170)
    //[-170, 239.5]
    double radar_object_relative_speed;

public:
    RadarStat() : Can(0x714, 8),
                  radar_object_state(0.0),
                  radar_object_lateral_pos(0.0),
                  radar_object_distant(0.0),
                  radar_object_relative_speed(0.0)
    {
    }
    ~RadarStat() {}
    void setValue(const unsigned char *_data)
    {
        setData(_data);
        radar_object_state = data[0];
        radar_object_lateral_pos = static_cast<double>((getTwoByte(data[2], data[3]) - 20)) * 0.1 - 18;
        radar_object_distant = static_cast<double>(getTwoByte(data[4], data[5])) * 0.1;
        radar_object_relative_speed = static_cast<double>((getTwoByte(data[6], data[7]) - 170)) - 1530;
        if (radar_object_state == 0)
        {
            radar_object_lateral_pos = 0;
            radar_object_distant = 0;
            radar_object_relative_speed = 0;
        }
    }
};
//차량에서 나오는 can data들
class VehicleData
{
public:
    EpsStat eps;
    AccStat acc;
    WheelSpeed wheel;
    YawAndBrake yaw_and_brake;
    RadarStat radar;
    double getSpeedDouble() { return (wheel.front_left + wheel.front_right + wheel.rear_left + wheel.rear_right) / 4.0; }
};
