#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <cmath>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>


#include "common/getopt.h"
#include "common/timestamp.h"
#include "a3_inverse_kinematics.h"
#include "math/angle_functions.hpp"    // This is where a lot of the internals live

#define NUM_SERVOS 6
const double pi = 3.1415926;

using namespace std;

InverseKinematics Arm;

static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    // Print out servo positions
    pthread_mutex_lock(&(Arm.lock));
    for (int id = 0; id < msg->len; id++) {
        Arm.real_angles[id] = msg->statuses[id].position_radians;
        // printf ("[id %d]=%6.3f ",id, Arm.real_angles[id]);
    }
    pthread_mutex_unlock(&(Arm.lock));
    // printf ("\n");
}

void *
status_loop (void *data)
{
    const int hz = 15;
    dynamixel_status_list_t_subscribe (Arm.lcm,
                                       Arm.status_channel,
                                       status_handler,
                                       &Arm);
    while (Arm.running) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until something is "ready" to happen. In this case, we are ready to
        // receive a message.
        int status = lcm_handle_timeout (Arm.lcm, 1000/hz);
        if (status <= 0)
            continue;
    }

    return NULL;
}

void InverseKinematics::move_to(double x, double y, double z, double wrist_tilt, bool dropping) {
    if (z < 0) {
        printf("z too low!\n");
        return;
    }
    // wrist_tilt = pi / 20.;
    //printf("Moving to %g, %g, %g\n", x, y, z);
    cout << "Moving to " << x << " " << y << " " << z << endl;
    // bool moving_horizontal = false;
    // if (abs(x - Arm.cmd_position[0]) + abs(Arm.cmd_position[1] - y) > 0.0001) {
    //     Arm.cmd_angles[1] = -pi/6;
    //     Arm.move_joints(Arm.cmd_angles);
    //     moving_horizontal = true;
    // }
    Arm.cmd_position[0] = x;
    Arm.cmd_position[1] = y;
    // Arm.cmd_position[2] = z;

    // if (!Arm.offset_position.empty()) {
    //     x += Arm.offset_position[0];
    //     y += Arm.offset_position[1];
    //     // z += Arm.offset_position[2];
    // }
    
    // Cheat Fix for arm.
    // if (x > 0)
    //     x = x + 0.1 * (y - 0.12);
    // else
    //     x = x - 0.1 * (y - 0.12);

    // y = y + abs(0.05 * x);

    // cout << "Corrected Moving to " << x << " " << y << " " << z << endl;
    
    vector<double> arm_length = {0.118, 0.1, 0.0985, 0.099}; // change this accordingly
    vector<double> max_angles = {pi,pi/12,pi/12,pi/12}; // change this accordingly
    

    double R = sqrt(pow(x, 2) + pow(y, 2));
    if (R > 0.3) {
        exit(-1);
    }
    // z += 0.003 / 0.03 * (R - 0.12369);
    Arm.cmd_angles[0] = eecs467::angle_sum(atan2(x, y), 0); // x and y reversed because angle begins from y, not x axis.
    
    // cout << "Theta is " << Arm.cmd_angles[0] << endl;

    // if (moving_horizontal)
    //     Arm.move_joints(Arm.cmd_angles);

    double M = sqrt(pow(R, 2) + pow(arm_length[3] + z - arm_length[0], 2));
    double alpha = atan2(arm_length[3] + z - arm_length[0], R);
    double beta  = acos((-pow(arm_length[2], 2) + pow(arm_length[1], 2) + pow(M, 2)) / (2 * arm_length[1] * M));
    double gamma = acos((+pow(arm_length[2], 2) + pow(arm_length[1], 2) - pow(M, 2)) / (2 * arm_length[1] * arm_length[2]));

    if (beta != beta || gamma != gamma) {
        beta = 0;
        gamma = pi;
        cout << "APPROXIMATIONNNNNNNNNNNNNNNNNNN" << endl;
    }

    Arm.cmd_angles[1] = pi/2 - alpha - beta;
    Arm.cmd_angles[2] = pi - gamma;
    Arm.cmd_angles[3] = pi - Arm.cmd_angles[1] - Arm.cmd_angles[2] - wrist_tilt;
    Arm.cmd_angles[4] = -pi/2. + Arm.cmd_angles[0] ;
    Arm.cmd_angles[5] = -pi/2.;

    // send angles command
    // Arm.cmd_angles[1] -= pi/24;
    // Arm.move_joints(Arm.cmd_angles, true);
    // Arm.cmd_angles[1] += pi/24;
    // Arm.move_joints(Arm.cmd_angles, true);
    // Arm.cmd_angles[1] -= pi/24;
    if (dropping) {
        Arm.cmd_angles[1] -= pi/24;
        Arm.move_joints(Arm.cmd_angles);
        Arm.cmd_angles[1] += pi/24;
    }
    Arm.move_joints(Arm.cmd_angles, true);
}

void InverseKinematics::transition_to(double x, double y, double z) {
    double tempX = x, tempY = y,tempZ;
    tempZ = z*2;
    if (x > .1) tempX = x/2;
    if (y > .1) tempY = y/2;
    move_to(tempX,tempY,z);
    usleep(750000);
    move_to(x,y,z);
}

void InverseKinematics::move_joints(vector<double> joint_angles, bool move_to) {
    // send angles command
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = (dynamixel_command_t*)calloc (NUM_SERVOS, sizeof(dynamixel_command_t));
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now ();
        cmds.commands[id].position_radians = -joint_angles[id];
        cmds.commands[id].speed = 0.08;
        cmds.commands[id].max_torque = 0.85;
    }
    // cmds.commands[5].position_radians *= -1;
    assert(Arm.lcm != NULL);
    dynamixel_command_list_t_publish (Arm.lcm, Arm.command_channel, &cmds);
      
      
    while(1) {
        double error = 0;
        pthread_mutex_lock(&(Arm.lock));
        for (int i = 0; i < NUM_SERVOS; ++i) {
            error += abs(Arm.real_angles[i] + joint_angles[i]);
            // printf("(%g, %g)\n", Arm.real_angles[i], joint_angles[i]);
        }
        pthread_mutex_unlock(&(Arm.lock));
        // cout << "error is " << error << endl;
        if (error < 0.15)
            break;

        usleep(10000);
    }
    
    // if (move_to) {
    //     vector<double> arm_length = {0.118, 0.1, 0.0985, 0.099}; // change this accordingly
    //     // cout << "real theta2 = " << -Arm.real_angles[1] << ", theta4 = " << -Arm.real_angles[3] << endl;
    //     double R = arm_length[1] * sin(-Arm.real_angles[1]) + arm_length[2] * sin(-Arm.real_angles[3]);
    //     // cout << "real R = " << R << endl;
    //     double real_x = R * sin(-Arm.real_angles[0]);
    //     double real_y = R * cos(-Arm.real_angles[0]);
    //     // double real_z = arm_length[0] + arm_length[1] * cos(-Arm.real_angles[1]) + arm_length[2] * cos(-Arm.real_angles[1]-Arm.real_angles[2])
    //     //                 + arm_length[3] * cos(-Arm.real_angles[1]-Arm.real_angles[2]-Arm.real_angles[2]);

    //     // cout << "real theta0 = " << Arm.real_angles[0] << endl;

    //     // cout << "Real x and y: " << real_x << ", " << real_y << endl;
    //     Arm.offset_position.clear();
    //     Arm.offset_position = {Arm.cmd_position[0] - real_x, Arm.cmd_position[1] - real_y};
    // }
    
    

    //cout << "Arm.move_joints(): Action completed." << endl;
    
}

void InverseKinematics::relax() {
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = (dynamixel_command_t*)calloc (NUM_SERVOS, sizeof(dynamixel_command_t));
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now ();
        cmds.commands[id].position_radians = 0;
        cmds.commands[id].speed = 0.00;
        cmds.commands[id].max_torque = 0;
    }
    // cmds.commands[5].position_radians *= -1;
    dynamixel_command_list_t_publish (Arm.lcm, Arm.command_channel, &cmds);
}

void InverseKinematics::fetch() {
    cout << "Fetching..." << endl;
    // lower claw
    cout << "Lowering Claw" << endl;
    move_to(Arm.cmd_position[0], Arm.cmd_position[1], 0.1);

    // straighten shoulder
    cout << "Closing Claw" << endl;

    Arm.cmd_angles[5] = -(pi/2);
    Arm.move_joints(Arm.cmd_angles);
    
    // raise claw
    cout << "Raising Claw" << endl;
    move_to(Arm.cmd_position[0], Arm.cmd_position[1], 0.13);
}

void InverseKinematics::drop() {
    cout << "Dropping" << endl;

    const double claw_rest_angle_c = -(pi/2 * 4/5.);
    Arm.cmd_angles[5] = claw_rest_angle_c;

    Arm.move_joints(Arm.cmd_angles);
}

void InverseKinematics::stand() {
    const double claw_rest_angle_c = -(pi/2 * 5/5.);
    vector<double> initial_joints = {0, 0, 0, pi/2, -pi/2., claw_rest_angle_c};
    // Arm.cmd_angles[0] = pi/4;
    // Arm.cmd_angles[1] = -pi/6;
    // Arm.move_joints(Arm.cmd_angles);
    // usleep(10000);
    // Arm.cmd_angles = initial_joints;
    // Arm.cmd_angles[0] = pi/4;
    // Arm.move_joints(Arm.cmd_angles);
    // usleep(10000);
    // Arm.cmd_angles = initial_joints;
    Arm.move_joints(initial_joints);
    // cout << "standing" << endl;
}

// This subscribes to the status messages sent out by the arm, displaying servo
// kin_state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int InverseKinematics::start (int argc, char *argv[])
{
    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");
    getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    Arm.gopt = gopt;
    Arm.lcm = lcm_create (NULL);
    Arm.command_channel = getopt_get_string (gopt, "command-channel");
    Arm.status_channel = getopt_get_string (gopt, "status-channel");

    pthread_create (&Arm.status_thread, NULL, status_loop, (void*)NULL);

    stand();

    // Probably not needed, given how this operates
    pthread_join (Arm.status_thread, NULL);
    // pthread_join (Arm.command_thread, NULL);

    lcm_destroy (Arm.lcm);
    getopt_destroy (gopt);
    return 0;
}
