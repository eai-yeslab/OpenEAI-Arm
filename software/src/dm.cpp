#include "dm.hpp"

#include "serial_port.hpp"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include <cmath>

float DM_Motor::Motor::get_param_as_float(int key) const
{
    auto it = param_map.find(key);
    if (it != param_map.end())
    {
        if (it->second.isFloat)
        {
            return it->second.value.floatValue;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

uint32_t DM_Motor::Motor::get_param_as_uint32(int key) const {
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (!it->second.isFloat) {
            return it->second.value.uint32Value;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

bool DM_Motor::Motor::is_have_param(int key) const
{
    return param_map.find(key) != param_map.end();
}


void DM_Motor::Motor_Control::enable(const Motor& motor)
{
    control_cmd(motor.GetSlaveId(), 0xFC);
    usleep(100000);//100ms
    this->receive();
}

void DM_Motor::Motor_Control::enable_old(const Motor& motor, Control_Mode mode)
{
    uint32_t id = ((mode -1) << 2) + motor.GetSlaveId();
    control_cmd(id, 0xFC);
    usleep(100000);
    this->receive();
}


void DM_Motor::Motor_Control::refresh_motor_status(const Motor& motor)
{
    uint32_t id = 0x7FF;
    uint8_t can_low = motor.GetSlaveId() & 0xff; // id low 8 bit
    uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; //id high 8 bit
    std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    this->receive();
}

void DM_Motor::Motor_Control::disable(const Motor& motor) {
    control_cmd(motor.GetSlaveId(), 0xFD);
    usleep(100000);
    this->receive();
}


void DM_Motor::Motor_Control::set_zero_position(const Motor& motor)
{
    control_cmd(motor.GetSlaveId(), 0xFE);
    usleep(100000);
    this->receive();
}


void DM_Motor::Motor_Control::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
{
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };
    Motor_id id = DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("Motor_Control id not found");
    }
    auto& m = motors[id];
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    Limit_param limit_param_cmd = m->get_limit_param();
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

    std::array<uint8_t, 8> data_buf{};
    data_buf[0] = (q_uint >> 8) & 0xff;
    data_buf[1] = q_uint & 0xff;
    data_buf[2] = dq_uint >> 4;
    data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    data_buf[4] = kp_uint & 0xff;
    data_buf[5] = kd_uint >> 4;
    data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    data_buf[7] = tau_uint & 0xff;

    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    this->receive();
}


void DM_Motor::Motor_Control::control_pos_vel(Motor &DM_Motor,float pos,float vel)
{
    Motor_id id = DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
    }
    std::array<uint8_t, 8> data_buf{};
    memcpy(data_buf.data(), &pos, sizeof(float));
    memcpy(data_buf.data() + 4, &vel, sizeof(float));
    id += POS_MODE;
    send_data.modify(id, data_buf.data());
    serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
    this->receive();
}

/*
    * @description: velocity control mode 速度控制模式
    * @param DM_Motor: motor object 电机对象
    * @param vel: velocity 速度
    */
void DM_Motor::Motor_Control::control_vel(Motor &DM_Motor,float vel)
{
    Motor_id id =DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("VEL ERROR : id not found");
    }
    std::array<uint8_t, 8> data_buf = {0};
    memcpy(data_buf.data(), &vel, sizeof(float));
    id=id+SPEED_MODE;
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    this->receive();
}

void DM_Motor::Motor_Control::control_pos_force(Motor &DM_Motor,float pos, uint16_t vel, uint16_t i)
{
    Motor_id id =DM_Motor.GetSlaveId();
    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("pos_force ERROR : Motor_Control id not found");
    }
    std::array<uint8_t, 8> data_buf{};
    memcpy(data_buf.data(), &pos, sizeof(float));
    memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
    memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
    id=id+POSI_MODE;
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    this->receive();
}

void DM_Motor::Motor_Control::receive()
{
    serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

    if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
    {
        static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
            float span = xmax - xmin;
            float data_norm = float(x) / ((1 << bits) - 1);
            float data = data_norm * span + xmin;
            return data;
        };

        auto & data = receive_data.canData;

        uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
        uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
        uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
        if(receive_data.canId != 0x00)   //make sure the motor id is not 0x00
        {
            if(motors.find(receive_data.canId) == motors.end())
            {
                return;
            }

            auto m = motors[receive_data.canId];
            Limit_param limit_param_receive = m->get_limit_param();
            float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
            m->receive_data(receive_q, receive_dq, receive_tau);
        }
        else //why the user set the masterid as 0x00 ???
        {
            uint32_t slaveID = data[0] & 0x0f;
            if(motors.find(slaveID) == motors.end())
            {
                return;
            }
            auto m = motors[slaveID];
            Limit_param limit_param_receive = m->get_limit_param();
            float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
            m->receive_data(receive_q, receive_dq, receive_tau);
        }
        return;
    }
    else if (receive_data.CMD == 0x01) // receive fail
    {
        /* code */
    }
    else if (receive_data.CMD == 0x02) // send fail
    {
        /* code */
    }
    else if (receive_data.CMD == 0x03) // send success
    {
        /* code */
    }
    else if (receive_data.CMD == 0xEE) // communication error
    {
        /* code */
    }
}

void DM_Motor::Motor_Control::receive_param()
{
    serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

    if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
    {
        auto & data = receive_data.canData;
        if(data[2]==0x33 or data[2]==0x55)
        {
            uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
            uint8_t RID = data[3];
            if (motors.find(slaveID) == motors.end())
            {
                //can not found motor id
                return;
            }
            if(is_in_ranges(RID))
            {
                uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                motors[slaveID]->set_param(RID, data_uint32);
            }
            else
            {
                float data_float = uint8_to_float(data + 4);
                motors[slaveID]->set_param(RID, data_float);
            }
        }
        return ;
    }
}

void DM_Motor::Motor_Control::addMotor(Motor *DM_Motor)
{
    motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
    if (DM_Motor->GetMasterId() != 0)
    {
        motors.insert({DM_Motor->GetMasterId(), DM_Motor});
    }
}

float DM_Motor::Motor_Control::read_motor_param(Motor &DM_Motor,uint8_t RID)
{
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(0x7FF, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            if (is_in_ranges(RID))
            {
                return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
            }
            else
            {
                return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
            }
        }
    }

    return 0;
}


bool DM_Motor::Motor_Control::switchControlMode(Motor &DM_Motor,Control_Mode mode)
{
    uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
    uint8_t RID = 10;
    write_motor_param(DM_Motor,RID,write_data);
    if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
    {
        return false;
    }
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
        }
    }
    return false;
}

bool DM_Motor::Motor_Control::change_motor_param(Motor &DM_Motor,uint8_t RID,float data)
{
    if(is_in_ranges(RID)) {
        //居然传进来的是整型的范围 救一下
        uint32_t data_uint32 = float_to_uint32(data);
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data_uint32;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    else
    {
        //is float
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
    {
        return false;
    }
    for(uint8_t i =0;i<max_retries;i++)
    {
        usleep(retry_interval);
        receive_param();
        if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
        {
            if (is_in_ranges(RID))
            {
                return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
            }
            else
            {
                return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) - data)<0.1f;
            }
        }
    }
    return false;
}


void DM_Motor::Motor_Control::save_motor_param(Motor &DM_Motor)
{
    disable(DM_Motor);
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
    send_data.modify(0x7FF, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    usleep(100000);//100ms wait for save
}


void DM_Motor::Motor_Control::control_cmd(Motor_id id , uint8_t cmd)
{
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
}

void DM_Motor::Motor_Control::write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
{
    uint32_t id = DM_Motor.GetSlaveId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
    data_buf[4] = data[0];
    data_buf[5] = data[1];
    data_buf[6] = data[2];
    data_buf[7] = data[3];
    send_data.modify(0x7FF, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
}





