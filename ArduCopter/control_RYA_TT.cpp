#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>

#include "Copter.h"

int server_socket;
int client_socket;
char client_mess[256];

int str_to_int(char *str);
int four_char_to_int(char *mess);

bool server_init(void);
bool TCP_decode(int &X_err_in_pixel, int &Y_err_in_pixel, char *str);

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::RYA_TT_init(bool ignore_checks)
{

    // init server TCP_IP: 127.0.0.1, port: 9000
    server_init();

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z())
    {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// should be called at 100hz or more
void Copter::RYA_TT_run()
{
    int X_err_in_pixel, Y_err_in_pixel;
    int height = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    float curr_height = (float)height;  // cm
    float curr_roll = ahrs.roll;        // rad
    float curr_pitch = ahrs.pitch;      // rad

    client_socket = accept(server_socket, NULL, NULL);
    recv(client_socket, &client_mess, sizeof(client_mess), 0);
    TCP_decode(X_err_in_pixel,Y_err_in_pixel,client_mess);

    cliSerial->printf("%f %f %f\n", curr_roll, curr_pitch, curr_height);
}

bool server_init(void){
    if(server_socket == 0){
        server_socket = socket(AF_INET, SOCK_STREAM,0);

        //define the server address
        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(9000);
        // server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_address.sin_addr.s_addr = INADDR_ANY;

        //bind the socket to our specified IP and port
        bind(server_socket, (struct sockaddr*) &server_address, sizeof(server_address));

        listen(server_socket, 5);

        fcntl(server_socket, F_SETFL, fcntl(server_socket, F_GETFL, 0) | O_NONBLOCK);
    }
}
bool TCP_decode(int &X_err_in_pixel, int &Y_err_in_pixel, char *str){
    char client_mess_x[5] = {0, 0, 0, 0, 0};
    char client_mess_y[5] = {0, 0, 0, 0, 0};
    int i = 0;
    for(i=0;i<=3;i++){
        client_mess_x[i] = str[i];
    }
    for(i=4;i<=7;i++){
        client_mess_y[i-4] = str[i];
    }
    X_err_in_pixel = four_char_to_int(client_mess_x);
    Y_err_in_pixel = four_char_to_int(client_mess_y);
    return true;
}
int four_char_to_int(char *mess){

    int i = 0, j = 0, flag = 0;
    int err = 0;
    while (i <= 3)
    {
        if (mess[i] == '-')
        {
            flag = 1;
            break;
        }
        i++;
    }

    char str[4] = {0, 0, 0, 0};

    if (flag == 0)
    {
        for (j = 0; j <= 2; j++)
        {
            str[j] = mess[j + 1];
        }
        err = str_to_int(str);
    }
    else
    {
        for (j = 0; j < 4 - i - 1; j++)
        {
            str[j] = mess[j + i + 1];
        }
        err = -str_to_int(str);
    }
    return err;
}
int str_to_int(char * str) {

    int num = 0;
    int i = 0;

    while (str[i] != 0) { // loop till there's nothing left
        char digit = str[i] - 48;
        num = 10 * num + digit; // "right shift" the number
        i++;
    }

    return num;
}
