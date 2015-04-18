#ifndef __A3_INVERSE_KINEMATICS__
#define __A3_INVERSE_KINEMATICS__

#include <vector>
#include <iostream>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

struct InverseKinematics
{
    getopt_t *gopt;

    // LCM
    lcm_t *lcm;
    const char *command_channel;
    const char *status_channel;
    pthread_t status_thread;
    pthread_t command_thread;
    int stages;
    std::vector<double> cmd_angles;
    std::vector<double> cmd_position;
    std::vector<double> offset_position;
    std::vector<double> real_angles;
    pthread_mutex_t lock;
    bool running;

    InverseKinematics() : stages(0), cmd_angles(6), cmd_position(3), real_angles(6), running(true), offset_position() {}
    int start (int argc, char *argv[]);

    void move_joints(std::vector<double> joint_angles, bool move_to = false);
    void move_to(double x, double y, double z, double wrist_tilt = 0);
    void transition_to(double x, double y, double z);
    void relax();
    void fetch();
    void drop();
    void stand();
};

extern InverseKinematics Arm;

#endif