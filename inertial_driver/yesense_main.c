#include     <stdio.h>      /*标准输入输出的定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*UNIX 标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>  
#include     <fcntl.h>	    /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <sys/time.h>
#include     <string.h>
#include     <getopt.h>
#include     "analysis_data.h"
#include     <bits/time.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#define MKDIR(path) mkdir(path, 0777)
#endif

// 辅助函数：递归创建目录
void create_directories(const char *file_path) {
    char temp[256];
    char *p = NULL;
    size_t len;

    snprintf(temp, sizeof(temp), "%s", file_path);
    len = strlen(temp);
    if (temp[len - 1] == '/') temp[len - 1] = 0;

    // 逐层遍历路径，遇到 '/' 则创建目录
    for (p = temp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (MKDIR(temp) != 0 && errno != EEXIST) {
                // 如果目录已存在，忽略错误；否则报错
            }
            *p = '/';
        }
    }
}


/*----------------------------------------------------------------------*/
#define TRUE 		1
#define FALSE 		-1
#define RX_BUF_LEN	512

/*----------------------------------------------------------------------*/
unsigned char g_recv_buf[512] = {0};
unsigned short g_recv_buf_idx = 0;
protocol_info_t g_output_info = {0};

char SENSOR_DATA_PATH[256] = "/home/orangepi/wendy/AirCraftEyeV6.1/yesense_decode_linux_v1.0/test.csv"; 
/*----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    char path[256] = "../save_data/infrared_data/infrared_data.txt";

    if (argc > 2) {
        strncpy(path, argv[1], sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
    }

    // 1. 先确保目录结构存在
    create_directories(path);

    int fd;
    int nread;
    char buffer[RX_BUF_LEN];
    char* dev  = NULL;
    struct termios oldtio,newtio;
    char new_dir[256] = {0};

    unsigned short cnt = 0;
    int pos = 0;
    int num=0;

    speed_t speed = B921600;
    dev = "/dev/ttyAMA0";	
    fd = open(dev, O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY); 
    if (fd < 0)	{
        printf("Can't Open Serial Port!\n");
        exit(0);	
    }
	
    printf("open serial port to decode msg!\n");

    //save to oldtio
    tcgetattr(fd, &oldtio);
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = IGNPAR;  
    newtio.c_oflag = 0;
    tcflush(fd,TCIFLUSH);  
    tcsetattr(fd,TCSAFLUSH,&newtio);  
    tcgetattr(fd,&oldtio);
	
    memset(buffer,0,sizeof(buffer));

    //打开文件
    // strcat(new_dir,SENSOR_DATA_PATH);  
    // strcat(new_dir,"/test.csv");  // 修改为.csv扩展名
    
    strcat(new_dir,path);  
    FILE *fp = fopen(new_dir, "w+");
    if (fp == NULL) {
        perror("Failed to open test.csv");
        pthread_exit(NULL);
    }
    
    // CSV表头 - 使用逗号分隔
    fprintf(fp, "num, time_stample_sec, time_stample_nsec, angle_rate.x, angle_rate.y, angle_rate.z,");
    fprintf(fp, "accel.x, accel.y, accel.z, quaternion0, quaternion1, quaternion2, quaternion3\n");

    fflush(fp);

    while(1)
    {
	nread = read(fd, buffer, RX_BUF_LEN);
	if(nread > 0)
	{
	    //printf("nread = %d\n", nread);
	    memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);             
	    g_recv_buf_idx += nread;
	}

        cnt = g_recv_buf_idx;
        pos = 0;
        if(cnt < YIS_OUTPUT_MIN_BYTES)
        {
            continue;
        }

        while(cnt > (unsigned int)0)
        {
            int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info);
            if(analysis_done == ret)	/*未查找到帧头*/
            {
                pos++;
                cnt--;
            }
            else if(data_len_err == ret)
            {
                break;
            }
            else if(crc_err == ret || analysis_ok == ret)	 /*删除已解析完的完整一帧*/
            {
                output_data_header_t *header = (output_data_header_t *)(g_recv_buf + pos);
                unsigned int frame_len = header->len + YIS_OUTPUT_MIN_BYTES;
                cnt -= frame_len;
                pos += frame_len;
                //memcpy(g_recv_buf, g_recv_buf + pos, cnt);

                if(analysis_ok == ret)
                {
                num++;
                // 获取系统实时时间（最精确）
                clock_gettime(CLOCK_REALTIME, &g_output_info.ts);

                fprintf(fp,"%d,%ld,%09ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    num,
                    g_output_info.ts.tv_sec, 
                    g_output_info.ts.tv_nsec,
                    g_output_info.angle_rate.x,
                    g_output_info.angle_rate.y,
                    g_output_info.angle_rate.z,
                    g_output_info.accel.x,
                    g_output_info.accel.y,
                    g_output_info.accel.z,
                    g_output_info.attitude.quaternion_data0,
                    g_output_info.attitude.quaternion_data1, 
                    g_output_info.attitude.quaternion_data2, 
                    g_output_info.attitude.quaternion_data3         
                );
            }
	    }
	}

        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
	tcflush(fd,TCIFLUSH);
	usleep(10000);
    }

    close(fd);
}


