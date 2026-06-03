#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <signal.h>

#include "analysis_data.h"

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#define MKDIR(path) mkdir(path, 0777)
#endif

#define TRUE        1
#define FALSE      -1
#define RX_BUF_LEN 512
#define RECV_BUF_LEN 4096

#define DEG2RAD (3.14159265358979323846 / 180.0)

static volatile int g_running = 1;

unsigned char g_recv_buf[RECV_BUF_LEN] = {0};
unsigned short g_recv_buf_idx = 0;
protocol_info_t g_output_info = {0};

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/*
 * Recursively create directories from file path
 * Example:
 * ../save_data/inertial_data/inertial_data.csv
 * will automatically create:
 * ../save_data/inertial_data
 */
void create_directories(const char *file_path)
{
    char temp[512];
    char *p = NULL;
    size_t len;

    if (file_path == NULL || strlen(file_path) == 0) {
        return;
    }

    snprintf(temp, sizeof(temp), "%s", file_path);
    len = strlen(temp);

    if (len == 0) {
        return;
    }

    if (temp[len - 1] == '/') {
        temp[len - 1] = '\0';
    }

    for (p = temp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';

            if (strlen(temp) > 0) {
                if (MKDIR(temp) != 0 && errno != EEXIST) {
                    perror("mkdir failed");
                }
            }

            *p = '/';
        }
    }
}

/*
 * Configure serial port:
 * 8 data bits
 * No parity
 * 1 stop bit
 * Raw mode
 */
int set_serial_port(int fd, speed_t speed)
{
    struct termios tio;

    if (tcgetattr(fd, &tio) != 0) {
        perror("tcgetattr failed");
        return -1;
    }

    cfmakeraw(&tio);

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_oflag &= ~OPOST;

    /*
     * Non-blocking read mode
     */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/wheeltec_IMU";
    speed_t speed = B921600;

    char path[512] = "../save_data/inertial_data/inertial_data.txt";

    int fd = -1;
    int nread = 0;
    char buffer[RX_BUF_LEN];

    unsigned short cnt = 0;
    int pos = 0;
    int num = 0;

    FILE *fp = NULL;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /*
     * Usage:
     * ./imu_record
     * ./imu_record ../save_data/inertial_data/test.csv
     */
    if (argc > 1) {
        strncpy(path, argv[1], sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
    }

    create_directories(path);

    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Cannot open serial port");
        printf("Device path: %s\n", dev);
        return -1;
    }

    printf("Open IMU serial port successfully.\n");
    printf("Device: %s\n", dev);
    printf("Baudrate: 921600\n");
    printf("Save path: %s\n", path);

    if (set_serial_port(fd, speed) != 0) {
        close(fd);
        return -1;
    }

    fp = fopen(path, "w+");
    if (fp == NULL) {
        perror("Failed to open output csv file");
        close(fd);
        return -1;
    }

    /*
     * CSV header
     */
    fprintf(fp, "num,time_stample_sec,time_stample_nsec,r_x,r_y,r_z,");
    fprintf(fp, "a_x,a_y,a_z,q_w,q_x,q_y,q_z\n");
    fflush(fp);

    memset(buffer, 0, sizeof(buffer));
    memset(g_recv_buf, 0, sizeof(g_recv_buf));
    g_recv_buf_idx = 0;

    while (g_running) {

        nread = read(fd, buffer, RX_BUF_LEN);

        if (nread > 0) {

            /*
             * Prevent receive buffer overflow
             */
            if ((int)g_recv_buf_idx + nread >= RECV_BUF_LEN) {
                fprintf(stderr, "Receive buffer overflow, clear buffer.\n");

                g_recv_buf_idx = 0;
                memset(g_recv_buf, 0, sizeof(g_recv_buf));

                continue;
            }

            memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);
            g_recv_buf_idx += nread;

        } else {

            usleep(1000);
        }

        cnt = g_recv_buf_idx;
        pos = 0;

        if (cnt < YIS_OUTPUT_MIN_BYTES) {
            continue;
        }

        while (cnt > 0) {

            int ret = analysis_data(g_recv_buf + pos,
                                    cnt,
                                    &g_output_info);

            if (analysis_done == ret) {

                /*
                 * Invalid frame header
                 * Skip one byte
                 */
                pos++;
                cnt--;

            } else if (data_len_err == ret) {

                /*
                 * Incomplete frame
                 * Wait for next serial data
                 */
                break;

            } else if (crc_err == ret || analysis_ok == ret) {

                output_data_header_t *header =
                    (output_data_header_t *)(g_recv_buf + pos);

                unsigned int frame_len =
                    header->len + YIS_OUTPUT_MIN_BYTES;

                if (frame_len == 0 || frame_len > cnt) {
                    break;
                }

                cnt -= frame_len;
                pos += frame_len;

                if (analysis_ok == ret) {

                    num++;

                    clock_gettime(CLOCK_REALTIME,
                                  &g_output_info.ts);

                    fprintf(fp,
                            "%d,%ld,%09ld,%.9f,%.9f,%.9f,"
                            "%.9f,%.9f,%.9f,"
                            "%.9f,%.9f,%.9f,%.9f\n",

                            num,

                            g_output_info.ts.tv_sec,
                            g_output_info.ts.tv_nsec,

                            /*
                             * Angular velocity
                             * Convert deg/s to rad/s
                             */
                            g_output_info.angle_rate.x * DEG2RAD,
                            g_output_info.angle_rate.y * DEG2RAD,
                            g_output_info.angle_rate.z * DEG2RAD,

                            /*
                             * Linear acceleration
                             */
                            g_output_info.accel.x,
                            g_output_info.accel.y,
                            g_output_info.accel.z,

                            /*
                             * Quaternion
                             */
                            g_output_info.attitude.quaternion_data0,
                            g_output_info.attitude.quaternion_data1,
                            g_output_info.attitude.quaternion_data2,
                            g_output_info.attitude.quaternion_data3);

                    /*
                     * Flush file every 50 frames
                     */
                    if (num % 50 == 0) {
                        fflush(fp);
                    }
                }

            } else {

                /*
                 * Unknown parser status
                 * Skip one byte
                 */
                pos++;
                cnt--;
            }
        }

        /*
         * Move remaining unparsed data
         * to the beginning of receive buffer
         */
        if (cnt > 0 && pos > 0) {

            memmove(g_recv_buf,
                    g_recv_buf + pos,
                    cnt);

        } else if (cnt == 0) {

            memset(g_recv_buf, 0, sizeof(g_recv_buf));
        }

        g_recv_buf_idx = cnt;

        /*
         * Do NOT call tcflush() in loop
         * Otherwise valid IMU data may be discarded
         */
        usleep(1000);
    }

    printf("\nStopping IMU recorder...\n");

    if (fp != NULL) {

        fflush(fp);
        fclose(fp);
        fp = NULL;
    }

    if (fd >= 0) {

        close(fd);
        fd = -1;
    }

    printf("Saved %d frames to: %s\n",
           num,
           path);

    return 0;
}

