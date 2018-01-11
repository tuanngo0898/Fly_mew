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
char client_mess[10];

bool server_init(void);
bool decode(char *msg, int &errX, int &errY);

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

int time = 0;
float pixel_per_cm = 0;
float X_err_in_cm = 0;
float Y_err_in_cm = 0;

#define MAX_CONTROL_ANGLE  133          // =1000/45*6 ___ 6o
#define MAX_ANGEL          0.0872664626 // 10o
#define KP         2
// should be called at 100hz or more
void Copter::RYA_TT_run()
{
    // Get current information
    int   height = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    float curr_height = (float)height;  // cm
    float curr_roll = ahrs.roll;        // rad
    float curr_pitch = ahrs.pitch;      // rad

    float target_roll, target_pitch;
    // get pilot desired lean angles
    // get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());
    // cliSerial->printf("target_roll_pitch_remote: %f %f\n", target_roll, target_pitch);

    if (curr_roll >= 0.79 || curr_pitch >= 0.79 || curr_roll <= -0.79 || curr_pitch <= -0.79){
        time++;
        // cliSerial->printf("%d \n", time);
        if (time >= 300)
        {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            init_disarm_motors();
        }
    }
    else time = 0;

    int X_err_in_pixel = 0, Y_err_in_pixel = 0;
    client_socket = accept(server_socket, NULL, NULL);
    recv(client_socket, &client_mess, sizeof(client_mess), 0);
    bool isThereaAnyObject = decode(client_mess, X_err_in_pixel, Y_err_in_pixel);

    decode(client_mess, X_err_in_pixel, Y_err_in_pixel);

    // cliSerial->printf("%f %f %f\n", curr_roll, curr_pitch, curr_height);
    // cliSerial->printf("client mess: %s \n",client_mess);
    // cliSerial->printf("err in pixel %d %d \n", X_err_in_pixel, Y_err_in_pixel);
    cliSerial->printf("high_roll_pitch_X_Y %f %f %f %d %d \n", curr_height, curr_roll, curr_pitch, X_err_in_pixel, Y_err_in_pixel);
    // Process information
    if (isThereaAnyObject && curr_roll < MAX_ANGEL && curr_roll >- MAX_ANGEL && curr_pitch < MAX_ANGEL && curr_pitch > -MAX_ANGEL ){
        pixel_per_cm = curr_height * 0.8871428438 * 2 / 800;
        X_err_in_cm = X_err_in_pixel * pixel_per_cm;
        Y_err_in_cm = Y_err_in_pixel * pixel_per_cm;

        target_roll = X_err_in_cm * KP;
        if (target_roll > MAX_CONTROL_ANGLE)
            target_roll = MAX_CONTROL_ANGLE;
        if (target_roll < -MAX_CONTROL_ANGLE)
            target_roll = -MAX_CONTROL_ANGLE;

        target_pitch = -Y_err_in_cm * KP;
        if (target_pitch > MAX_CONTROL_ANGLE)
            target_pitch = MAX_CONTROL_ANGLE;
        if (target_pitch < -MAX_CONTROL_ANGLE)
            target_pitch = -MAX_CONTROL_ANGLE;
    }
    else{
        target_roll = 0;
        target_pitch = 0;
    }

    // cliSerial -> printf(" real target roll, pitch: %f  %f \n",target_roll,target_pitch);

    // Use information
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    // float target_roll, target_pitch;
    // get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock())
    {
        althold_state = AltHold_MotorStopped;
    }
    else if (takeoff_state.running || takeoff_triggered)
    {
        althold_state = AltHold_Takeoff;
    }
    else if (!ap.auto_armed || ap.land_complete)
    {
        althold_state = AltHold_Landed;
    }
    else
    {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state)
    {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running)
        {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f)
        {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        }
        else
        {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok())
        {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
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
    return true;
}

bool decode(char* msg, int& errX, int& errY){
    if (msg[0] == '0')
    {
        errX = 0;
        errY = 0;
        return false;
    }
    else{
        errX = 0;
        errX += (int)(msg[2] - 48) * 100;
        errX += (int)(msg[3] - 48) * 10;
        errX += (int)(msg[4] - 48);
        if (msg[1] == '1') errX = - errX;

        errY = 0;
        errY += (int)(msg[6] - 48) * 100;
        errY += (int)(msg[7] - 48) * 10;
        errY += (int)(msg[8] - 48);
        if (msg[5] == '1')errY = - errY;
        
        return true;
    }
}
