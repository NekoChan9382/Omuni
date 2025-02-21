#include <mbed.h>


/// @Tips mm rad s
#include <PID_new.hpp>
#include <cmath>
#include <array>

BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial controller(PA_9, PA_10, 115200);
CAN can(PA_11, PA_12, 1000000);
CANMessage msg_encoder;
CANMessage msg_motor;

PidGain pid_gain = {150.0, 900.0, 0.0};

bool readline(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);

int main()
{
    constexpr int motor_amount = 4;
    constexpr int ppr = 256;
    int velocity_xy[2] = {0}; // x, y, ang
    float velocity_ang = 0;
    int pos_mm[3] = {0}; // x, y, ang
    int encoder_diff[motor_amount] = {0};
    int encoder_diff_pre[motor_amount] = {0};
    constexpr int wheel_radius = 34; // mm
    constexpr int robot_size_calc = 190;  // mm
    constexpr int robot_size_motor = 240;

    std::array<Pid, motor_amount> pid = {Pid({pid_gain, 10000, -10000}),
                                        Pid({pid_gain, 10000, -10000}),
                                        Pid({pid_gain, 10000, -10000}),
                                        Pid({pid_gain, 10000, -10000})};

    for (int i = 0; i < motor_amount; i++)
    {
        pid[i].reset();
    }

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
                encoder_diff[i] += encoder_data[i] - encoder_diff_pre[i];
                encoder_diff_pre[i] = encoder_data[i];
            }
        }
        char data[10] = "";
        if (readline(controller, data) == 0)
        {
            printf("%s\n", data);

            if (strcmp(data, "trans") == 0)
            {
                printf("trans\n");
                for (int i = 0; i < 2; i++)
                {
                    char data_trans[10] = "";
                    if (readline(controller, data_trans, true) == 0)
                    {
                        velocity_xy[i] = atoi(data_trans) * 20;
                    }
                }
            }
            if (strcmp(data, "rot") == 0)
            {
                char data_rot[10] = "";
                if (readline(controller, data_rot, false, true) == 0)
                {
                    velocity_ang = atof(data_rot) * M_PI / 180 * -2;
                    // printf("velocity_ang: %f\n", velocity_ang);
                }
            }
        }
        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("encoder_diff: %d, %d, %d, %d\n", encoder_diff[0], encoder_diff[1], encoder_diff[2], encoder_diff[3]);
            int motor_dps[motor_amount];
            for (int i = 0; i < motor_amount; ++i)
            {
                motor_dps[i] = encoder_diff[i] / elapsed;
                encoder_diff[i] = 0;
            }
            int16_t motor_output[motor_amount] = {0};
            float theta_rad = atan2(velocity_xy[1], velocity_xy[0]);
            float output_power = hypot(velocity_xy[0], velocity_xy[1]);
            printf("output_power: %f\n", output_power);
            for (int i = 0; i < motor_amount; i++)
            {
                float enc_to_rad = 2 * M_PI / ppr * 2;
                float motor_ang = i * M_PI / 2;
                float motor_offset = M_PI / 4;
                float goal_ang_vel = (sin(theta_rad - (motor_ang + motor_offset)) * output_power + velocity_ang * robot_size_motor) / wheel_radius;
                printf("goal_ang_vel %d: %f\n", i, goal_ang_vel);
                motor_output[i] = pid[i].calc(goal_ang_vel, motor_dps[i] * enc_to_rad, elapsed);
            }
            // printf("motor_output: %d, %d, %d, %d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            printf("motor_dps: %d, %d, %d, %d\n", motor_dps[0], motor_dps[1], motor_dps[2], motor_dps[3]);
            CANMessage msg_motor(2, (const uint8_t *)motor_output, 8);
            can.write(msg_motor);
            pre = now;
        }
    }
}

bool readline(BufferedSerial &serial, char *buffer, const bool is_integar, const bool is_float)
{
    int i = 0;        // 繰り返し変数
    char buff = '0';  // シリアル受信

    if (not serial.readable())
    {
        return 1;
    }

    while ((buff != '\n') and i < 10)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信
        // printf("%c", buff);

        if (buff != '\n' && buff != '\r')
        {
            buffer[i] = buff; // 受信データ保存

            if (is_integar)
            {

                if (((buff < '0' || buff > '9') && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
            if (is_float)
            {

                if (((buff < '0' || buff > '9') && buff != '.' && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
        }
        i++;
    }
    // printf("\n");
    return 0;
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}