#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "serial.h"
#include "dvl_protocol.h"
#include <ctype.h> //isdigit()



uint64_t get_now_time() {
  struct timespec spec;
  if (clock_gettime(1, &spec) == -1) { /* 1 is CLOCK_MONOTONIC */
    abort();
  }

  return spec.tv_sec * 1000 + spec.tv_nsec / 1e6;
}


int chartoint(char *s)
{

    int i, n;
    n = 0;
    for (i = 0; isdigit(s[i]); ++i){
        n = 10 * n + (s[i] - '0');
    }
    return n; 
}

int main()
{
    printf("\nA sample C program\n\n");

    const char * device = "/dev/ttyUSB0";
    uint32_t baud_rate = 115200;

    
    int fd = open_serial_port(device, baud_rate);
    if (fd < 0) { return 1; }

    //Dynamic memory allocation structures 
    struct Element *arr_wrz = malloc(sizeof(struct Element) * 19);
    struct Element *arr_wru = malloc(sizeof(struct Element) * 5);
    struct Element *arr_wrp = malloc(sizeof(struct Element) * 9);
    
    create_wrz_struct(arr_wrz);
    create_wru_struct(arr_wru);
    create_wrp_struct(arr_wrp);

    //Structure for velocity report
    struct wrz_t wrz;
    //Structure for transductors
    struct wru_t wru0;
    struct wru_t wru1;
    struct wru_t wru2;
    struct wru_t wru3;
    //Structure for dead reckoning report
    struct wrp_t wrp;
    

    uint8_t sync_data = 0;
    uint8_t transductorsReady = 0;
    while(1)
    {
        int length = 0;
        uint8_t str[180];
        uint8_t checksum[3];
        uint8_t buffer = '\0';
        int count = 0;

        memset(checksum, '\0', sizeof(checksum));
        memset(str, '\0', sizeof(str));

        //Read data from serial port
        do
        {
            length = read(fd, &buffer, 1);
            sprintf(&str[count], "%c", buffer);
            count = count + length;
            //fprintf(stderr, "lenght: %d time: %ld got %c\n", length, time, buffer);

        }while(buffer != '\n' && length > 0);

        if(str != NULL)
        {
            //Get checksum from incoming data 
            sprintf(&checksum[0], "%c", str[count-4]);
            sprintf(&checksum[1], "%c", str[count-3]);



            //Calculate checksum from incoming data
            //uint8_t *str_crc = "wru,0,-0.005,0.06,-37,-83*c3";
            uint8_t crc = crc8(str, count - 5);

            //printf("count: %d\n", count);
            uint64_t time = get_now_time();
            //fprintf(stderr, "\ntime: %ld got %s", time,  str);

            /***
             * Velocity report (wrz)
             * Format: wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[covariance],[time_of_validity],[time_of_transmission],[time],[status] 
             * # Status -> 0: normal operation, 1 for operational issues such as hight temperature
             * Transducer report (wru)
             * Format: wru,[id],[velocity],[distance],[rssi],[nsd]
             * Dead reckoning report (wrp)
             * Format: wrp,[time_stamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],[status]
            ***/

            //compare the calculated checksum with that of the received information
            uint8_t num = strtol(checksum, NULL, 16);
            if(crc == num && (crc || num != 0))
            {
                printf("time: %ld Calculated checksum: %03x Received checksum: %03x\n", time, crc, num);

                if(split_message((char*)str, count, "wrz", arr_wrz))
                {
                    transductorsReady = 0;
                    wrz.velocity.x = *((double*)(arr_wrz[0].data));
                    wrz.velocity.y = *((double*)(arr_wrz[1].data));
                    wrz.velocity.z = *((double*)(arr_wrz[2].data));
                    wrz.valid = (char*)arr_wrz[3].data;
                    wrz.altitude = *((double*)(arr_wrz[4].data));
                    wrz.fom = *((double*)(arr_wrz[5].data));
                    for(int i = 0; i < 9; i++)
                        wrz.covariance[i] = *((double*)(arr_wrz[i+6].data));
                    wrz.time_of_validity = *((long*)(arr_wrz[16].data));
                    wrz.time_of_transmission = *((long*)(arr_wrz[17].data));
                    wrz.status = (int)*((long*)(arr_wrz[18].data));
                }

                if(split_message((char*)str, count, "wru", arr_wru))
                {
                    transductorsReady = 0;
                    switch (*((long*)(arr_wru[0].data)))
                    {
                    case 0:
                        wru0.id = *((long*)(arr_wru[0].data));
                        wru0.velocity = *((double*)(arr_wru[1].data));
                        wru0.distance = *((double*)(arr_wru[2].data));
                        wru0.rssi = *((long*)(arr_wru[3].data));
                        wru0.nsd = *((long*)(arr_wru[4].data));
                        break;
                    
                    case 1:
                        wru1.id = *((long*)(arr_wru[0].data));
                        wru1.velocity = *((double*)(arr_wru[1].data));
                        wru1.distance = *((double*)(arr_wru[2].data));
                        wru1.rssi = *((long*)(arr_wru[3].data));
                        wru1.nsd = *((long*)(arr_wru[4].data));
                        break;

                    case 2:
                        wru2.id = *((long*)(arr_wru[0].data));
                        wru2.velocity = *((double*)(arr_wru[1].data));
                        wru2.distance = *((double*)(arr_wru[2].data));
                        wru2.rssi = *((long*)(arr_wru[3].data));
                        wru2.nsd = *((long*)(arr_wru[4].data));
                        break;

                    case 3:
                        wru3.id = *((long*)(arr_wru[0].data));
                        wru3.velocity = *((double*)(arr_wru[1].data));
                        wru3.distance = *((double*)(arr_wru[2].data));
                        wru3.rssi = *((long*)(arr_wru[3].data));
                        wru3.nsd = *((long*)(arr_wru[4].data));
                        transductorsReady = 1;
                        break;
                    
                    default:
                        break;
                    }
                    
                }

                //if(transductorsReady)
                //    printf("t3: %d\n", wru3.id);

                if(split_message((char*)str, count, "wrp", arr_wrp))
                {
                    wrp.time_stamp = *((double*)(arr_wrp[0].data));
                    wrp.position.x = *((double*)(arr_wrp[1].data));
                    wrp.position.y = *((double*)(arr_wrp[2].data));
                    wrp.position.z = *((double*)(arr_wrp[3].data));
                    wrp.pos_std = *((double*)(arr_wrp[4].data));
                    wrp.roll = *((double*)(arr_wrp[5].data));
                    wrp.pitch = *((double*)(arr_wrp[6].data));
                    wrp.yaw = *((double*)(arr_wrp[7].data));
                    wrp.status = (int)*((long*)(arr_wrp[8].data));
                }

                /** 
                 * Configuration over serial 
                **/

                //Reset dead reckoning (wcr)
                //uint8_t *cmd = "wcr\n";
                //write(fd, cmd, sizeof(cmd) -1);

                //Calibrate gyro (wcg)
                //uint8_t *cmd = "wcg\n";
                //write(fd, cmd, sizeof(cmd) -1);


                //Is the dead reckoning reset successful?
                switch (is_dead_reckoning_reset_successful(str))
                {
                case 0:
                    printf("Reset is not successful!\n");
                    break;
                case 1:
                    printf("Reset is successful!\n");
                    break;             
                default:
                    break;
                }
                    


                printf("\n");
            }

        }


        //usleep(10);
    }

    free_struct(arr_wrz, 19);
    free_struct(arr_wru, 5);
    free_struct(arr_wrp, 9);

    close(fd);
    return 0;
}