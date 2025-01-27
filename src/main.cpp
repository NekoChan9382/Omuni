#include <mbed.h>

/// @Tips mm rad s
#include <PID_new.hpp>
#include <cmath>
#include <array>
#include <C620.hpp>

bool readline(BufferedSerial &serial, char *buffer, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);

int main()
{
    constexpr int pid_max = 1;
    constexpr int dji_max_output = 8000;
    constexpr int motor_amount = 4;
    int velocity_xy[2] = {0}; // x, y, ang
    float velocity_ang = 0;
    int pos_mm[3] = {0}; // x, y, ang
    constexpr int wheel_radius = 50; // mm
    constexpr int robot_size = 300;  // mm

    BufferedSerial pc(USBTX, USBRX, 115200);
    BufferedSerial controller(PA_9, PA_10, 115200);
    dji::C620 c620(PB_12, PB_13);

    PidGain pid_gain = {0.001, 0.00001, 0.0};
    std::array<Pid, motor_amount> pid = {Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max}),
                                         Pid({pid_gain, -pid_max, pid_max})};

    for (int i = 0; i < motor_amount; i++)
    {
        pid[i].reset();
    }

    c620.set_max_output(dji_max_output);

    while (true)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        c620.read_data();

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
                        velocity_xy[i] = atoi(data_trans);
                    }
                }
            }
            if (strcmp(data, "rot") == 0)
            {
                char data_rot[10] = "";
                if (readline(controller, data_rot, false, true) == 0)
                {
                    velocity_ang = atof(data_rot) * M_PI / 180 * 0.3;
                    // printf("velocity_ang: %f\n", velocity_ang);
                }
            }
        }
        if (now - pre > 30ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("encoder_diff: %d, %d, %d, %d\n", encoder_diff[0], encoder_diff[1], encoder_diff[2], encoder_diff[3]);

            int16_t motor_output[motor_amount] = {0};
            float theta_rad = atan2(velocity_xy[1], velocity_xy[0]);
            float output_power = hypot(velocity_xy[0], velocity_xy[1]);
            // printf("output_power: %f\n", output_power);
            for (int i = 0; i < motor_amount; i++)
            {
                float rmp_to_rad = 2 * M_PI / 60;
                float motor_dps = c620.get_rpm(i + 1) * rmp_to_rad;
                float motor_ang = i * M_PI / 2;
                float motor_offset = M_PI / 4;
                float goal_ang_vel = (sin(theta_rad - (motor_ang + motor_offset)) * output_power + velocity_ang * robot_size) / wheel_radius;
                goal_ang_vel *=  200;
                printf("goal_ang_vel %d: %f, dps: %f\n", i, goal_ang_vel, motor_dps);
                const float out = pid[i].calc(goal_ang_vel, motor_dps, elapsed);
                printf("out: %f\n", out);
                c620.set_output_percent(out, i + 1);
                // c620.set_output(0, i+1);
            }
            for (int i = 0; i < motor_amount; i++)
            {
                motor_output[i] = c620.get_current(i + 1);
            }
            // printf("motor_output: %d, %d, %d, %d\n", motor_output[0], motor_output[1], motor_output[2], motor_output[3]);
            // printf("motor_dps: %d, %d, %d, %d\n", motor_dps[0], motor_dps[1], motor_dps[2], motor_dps[3]);
            c620.write();
            pre = now;
        }
    }
}

bool readline(BufferedSerial &serial, char *buffer, const bool is_integar, const bool is_float)
{
    int i = 0;       // 繰り返し変数
    char buff = '0'; // シリアル受信

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