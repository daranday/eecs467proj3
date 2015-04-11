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

kin_state_t kin_state_obj;
kin_state_t *kin_state = &kin_state_obj;

static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    // Print out servo positions
    pthread_mutex_lock(&(kin_state->arm_lock));
    for (int id = 0; id < msg->len; id++) {
        kin_state->arm_joints[id] = msg->statuses[id].position_radians;
        // printf ("[id %d]=%6.3f ",id, kin_state->arm_joints[id]);
    }
    pthread_mutex_unlock(&(kin_state->arm_lock));
    // printf ("\n");
}

void *
status_loop (void *data)
{
    const int hz = 15;
    dynamixel_status_list_t_subscribe (kin_state->lcm,
                                       kin_state->status_channel,
                                       status_handler,
                                       kin_state);
    while (kin_state->running) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until something is "ready" to happen. In this case, we are ready to
        // receive a message.
        int status = lcm_handle_timeout (kin_state->lcm, 1000/hz);
        if (status <= 0)
            continue;
    }

    return NULL;
}

void move_to(double x, double y, double z, double wrist_tilt) {
    if (z < 0) {
        printf("z too low!\n");
        return;
    }
    // wrist_tilt = pi / 20.;
    printf("Moving to %g, %g, %g\n", x, y, z);
    // bool moving_horizontal = false;
    // if (abs(x - kin_state->cmd_position[0]) + abs(kin_state->cmd_position[1] - y) > 0.0001) {
    //     kin_state->cmd_angles[1] = -pi/6;
    //     move_joints(kin_state->cmd_angles);
    //     moving_horizontal = true;
    // }
    kin_state->cmd_position[0] = x;
    kin_state->cmd_position[1] = y;

    
    vector<double> arm_length = {0.118, 0.1, 0.0985, 0.099}; // change this accordingly
    vector<double> max_angles = {pi,pi/12,pi/12,pi/12}; // change this accordingly
    

    double R = sqrt(pow(x, 2) + pow(y, 2));
    // z += 0.003 / 0.03 * (R - 0.12369);
    kin_state->cmd_angles[0] = eecs467::angle_sum(atan2(x, y), 0); // x and y reversed because angle begins from y, not x axis.

    // if (moving_horizontal)
    //     move_joints(kin_state->cmd_angles);

    double M = sqrt(pow(R, 2) + pow(arm_length[3] + z - arm_length[0], 2));
    double alpha = atan2(arm_length[3] + z - arm_length[0], R);
    double beta  = acos((-pow(arm_length[2], 2) + pow(arm_length[1], 2) + pow(M, 2)) / (2 * arm_length[1] * M));
    double gamma = acos((+pow(arm_length[2], 2) + pow(arm_length[1], 2) - pow(M, 2)) / (2 * arm_length[1] * arm_length[2]));

    if (beta != beta || gamma != gamma) {
        beta = 0;
        gamma = pi;
    }

    // printf("M: %g, alpha: %g, beta0: %g, gamma0: %g, R: %g\n", M, alpha, (-pow(arm_length[2], 2) + pow(arm_length[1], 2) + pow(M, 2)) / (2 * arm_length[1] * M), (+pow(arm_length[2], 2) + pow(arm_length[1], 2) - pow(M, 2)) / (2 * arm_length[1] * arm_length[2]), R);
    // printf("d2: %g, d3: %g, M: %g\n", arm_length[1], arm_length[2], M);

    kin_state->cmd_angles[1] = pi/2 - alpha - beta;
    kin_state->cmd_angles[2] = pi - gamma;
    kin_state->cmd_angles[3] = pi - kin_state->cmd_angles[1] - kin_state->cmd_angles[2] - wrist_tilt;
    kin_state->cmd_angles[4] = pi/2.;
    kin_state->cmd_angles[5] = -pi/2.;

    // send angles command
    move_joints(kin_state->cmd_angles);
}

void move_to_smooth(double x, double y, double z, double wrist_tilt) {
    double tempX = x, tempY = y,tempZ;
    tempZ = z*2;
    if (x > .1) tempX = x/2;
    if (y > .1) tempY = y/2;
    move_to(tempX,tempY,z);
    usleep(750000);
    move_to(x,y,z);
}

void move_joints(vector<double> joint_angles) {
    // send angles command
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = (dynamixel_command_t*)calloc (NUM_SERVOS, sizeof(dynamixel_command_t));
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now ();
        cmds.commands[id].position_radians = -joint_angles[id];
        cmds.commands[id].speed = 0.05;
        cmds.commands[id].max_torque = 0.75;
    }
    // cmds.commands[5].position_radians *= -1;
    assert(kin_state->lcm != NULL);
    dynamixel_command_list_t_publish (kin_state->lcm, kin_state->command_channel, &cmds);
        /*
    while(1) {
        double error = 0;
        pthread_mutex_lock(&(kin_state->arm_lock));
        for (int i = 0; i < NUM_SERVOS; ++i) {
            error += abs(kin_state->arm_joints[i] + joint_angles[i]);
            printf("(%g, %g)\n", kin_state->arm_joints[i], joint_angles[i]);
        }
        pthread_mutex_unlock(&(kin_state->arm_lock));
        cout << "error is " << error << endl;
        if (error < 0.15)
            break;
        usleep(10000);
    }
    cout << "move_joints(): Action completed." << endl;
    */
}

void relax_arm() {
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
    dynamixel_command_list_t_publish (kin_state->lcm, kin_state->command_channel, &cmds);
}

void arm_fetch() {
    cout << "Fetching..." << endl;
    // lower claw
    cout << "Lowering Claw" << endl;
    move_to(kin_state->cmd_position[0], kin_state->cmd_position[1], 0.1);

    // straighten shoulder
    cout << "Closing Claw" << endl;

    kin_state->cmd_angles[5] = -(pi/2);
    move_joints(kin_state->cmd_angles);
    
    // raise claw
    cout << "Raising Claw" << endl;
    move_to(kin_state->cmd_position[0], kin_state->cmd_position[1], 0.13);
}

void arm_drop() {
    cout << "Dropping" << endl;

    const double claw_rest_angle_c = -(pi/2 * 4/5.);
    kin_state->cmd_angles[5] = claw_rest_angle_c;

    move_joints(kin_state->cmd_angles);
}

void stand_arm() {
    const double claw_rest_angle_c = -(pi/2 * 5/5.);
    vector<double> initial_joints = {0, 0, 0, 0, pi/2., claw_rest_angle_c};
    // kin_state->cmd_angles[0] = pi/4;
    // kin_state->cmd_angles[1] = -pi/6;
    // move_joints(kin_state->cmd_angles);
    // usleep(10000);
    // kin_state->cmd_angles = initial_joints;
    // kin_state->cmd_angles[0] = pi/4;
    // move_joints(kin_state->cmd_angles);
    // usleep(10000);
    // kin_state->cmd_angles = initial_joints;
    move_joints(initial_joints);
    // cout << "standing" << endl;
}

// This subscribes to the status messages sent out by the arm, displaying servo
// kin_state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int kin_main (int argc, char *argv[])
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


    kin_state->gopt = gopt;
    kin_state->lcm = lcm_create (NULL);
    kin_state->command_channel = getopt_get_string (gopt, "command-channel");
    kin_state->status_channel = getopt_get_string (gopt, "status-channel");

    pthread_create (&kin_state->status_thread, NULL, status_loop, (void*)NULL);


    // vector<double> initial_joints = {0, pi/2, 0,0,0,0};
    // vector<double> initial_joints = {0.0713075, 0.565071, 0.513557, 2.06296, 0, claw_rest_angle_c};
    stand_arm();

    // double interval_x = 0.065, interval_y = 0.065;
    // double origin_x = 0.01 + interval_x , origin_y = 0.13 - interval_y;

    // relax_arm();
    // pthread_create (&kin_state->command_thread, NULL, command_loop, kin_state);

    // while(1) {
    //     double a, b;
    //     string cmd;
    //     cout << "command > ";
    //     cin >> cmd;
    //     if (cmd == "mv") {
    //         cin >> a >> b;
    //         kin_state->cmd_position = {-a * interval_x + origin_x, b * interval_y + origin_y, 0.13};
    //         // kin_state->cmd_position = {a, b, 0.15};
    //         kin_state->cmd_angles[1] = -pi/6;
    //         move_joints(kin_state->cmd_angles);
    //         move_to(kin_state->cmd_position[0], kin_state->cmd_position[1], 0.13);
    //     } else if (cmd == "fetch") {
    //         cout << "Fetching" << endl;
    //         // lower claw
    //         move_to(kin_state->cmd_position[0], kin_state->cmd_position[1], 0.1);

    //         // straighten shoulder
    //         kin_state->cmd_angles[5] = -(pi/2);
    //         move_joints(kin_state->cmd_angles);
            
    //         // raise claw
    //         move_to(kin_state->cmd_position[0], kin_state->cmd_position[1], 0.13);
    //     } else if (cmd == "drop") {
    //         cout << "Dropping" << endl;
    //         kin_state->cmd_angles[5] = claw_rest_angle_c;
    //         move_joints(kin_state->cmd_angles);
    //     } else if (cmd == "quit") {
    //         kin_state->running = false;
    //         break;
    //     }
    // }


    // Probably not needed, given how this operates
    pthread_join (kin_state->status_thread, NULL);
    // pthread_join (kin_state->command_thread, NULL);

    lcm_destroy (kin_state->lcm);
    free (kin_state);
    getopt_destroy (gopt);
}
