#include <mbed.h>
#include <PID_new.hpp>
#include <cstring>
#include <math.h>

BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial controller(PA_9, PA_10, 115200);
CAN can(PA_11, PA_12, 1000000);

PidGain pid_gain = {2400.0, 0.0, 0.0};
Pid pid({pid_gain, 24000, -24000}); // P,I,D,Max,Min

bool readlines(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false);

constexpr float rad_to_deg = 180.0 / M_PI;
constexpr float deg_to_rad = M_PI / 180.0;

int main()
{
    CANMessage msg_encoder;
    CANMessage msg_motor;

    constexpr int motor_amount = 4;
    constexpr int ppr = 256;
    constexpr int motor_place_angle[motor_amount] = {45, 135, 225, 315}; // degree
    constexpr float wheel_radius = 0.03; // meter
    constexpr float center_to_wheel = 1.0; // meter
    int motor_theta[motor_amount] = {0}; // degree
    int pre_motor_theta[motor_amount] = {0}; // degree
    int robot_velocity[3] = {0}; // m/s, m/s, deg/s
    int goal_angular_velocity[motor_amount] = {0}; // deg/s
    int motor_output[motor_amount] = {0};

    while (true)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        if (can.read(msg_encoder); msg_encoder.id == 10)
        {
            int16_t encoder_data[motor_amount];
            for (int i = 0; i < motor_amount; i++)
            {
                encoder_data[i] = msg_encoder.data[2 * i + 1] << 8 | msg_encoder.data[2 * i];
                float encoder_to_deg = 360.0 / (ppr * 2.0);
                motor_theta[i] = encoder_data[i] * encoder_to_deg;
            }
        }
        char data[10];
        if (readlines(controller, data) == 0)
        {
            if (strcmp(data, "trans\n") == 0)
            {
                for (int i = 0; i < 2; i++)
                {
                    if (readlines(controller, data, true) == 0)
                    {
                        robot_velocity[i] = atoi(data);
                    }
                }
            }
            if (strcmp(data, "rot\n") == 0)
            {
                if (readlines(controller, data, true) == 0)
                {
                    robot_velocity[2] = atoi(data);
                }
            }
        }
        if (now - pre > 10ms)
        {
            int motor_dps[motor_amount];
            for (int i = 0; i < motor_amount; i++)
            {
                if (motor_theta[i] - pre_motor_theta[i] > 32767)
                {
                    motor_theta[i] += 65535; // -32767 + k = 32768
                }
                motor_dps[i] = (motor_theta[i] - pre_motor_theta[i]) * 100; // * 100 = / 0.01
                pre_motor_theta[i] = motor_theta[i];

                goal_angular_velocity[i] = (sin(motor_place_angle[i] * deg_to_rad) * robot_velocity[0] + cos(motor_place_angle[i] * deg_to_rad) * robot_velocity[1] + robot_velocity[2] * center_to_wheel) / wheel_radius * rad_to_deg;
                motor_output[i] = pid.calc(goal_angular_velocity[i], motor_dps[i], 0.01);
            }
            CANMessage msg_motor(2, (const uint8_t *)motor_output, 8);
            can.write(msg_motor);
            pre = now;
        }
    }
}

bool readlines(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false)
{
    int i = 0;        // 繰り返し変数
    char buff = '0';  // シリアル受信
    string data = ""; // 受信データ保存

    if (not serial.readable())
    {
        return 1;
    }

    while ((buff != '\n') and i < 10)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信

        if (buff != '\n' && buff != '\r')
        {
            data += buff; // 受信データ保存

            if (is_integar)
            {

                if ((buff < '0' || buff > '9'))
                {
                    printf("error\n");
                    return 1;
                }
            }
            if (is_float)
            {

                if ((buff < '0' || buff > '9' || buff != '.'))
                {
                    printf("error\n");
                    return 1;
                }
            }
        }
        i++;
    }
    return 0;
}