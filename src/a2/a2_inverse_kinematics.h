#ifndef __A2_INVERSE_KINEMATICS__
#define __A2_INVERSE_KINEMATICS__

#include <vector>
#include <iostream>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

struct kin_state_t
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

    std::vector<double> arm_joints;
    pthread_mutex_t arm_lock;
    bool running;
    kin_state_t() : stages(0), cmd_angles(6), cmd_position(3), arm_joints(6), running(true) {}
};

extern kin_state_t kin_state_obj;
extern kin_state_t *kin_state;

void move_joints(std::vector<double> joint_angles);
void move_to(double x, double y, double z, double wrist_tilt = 0);
int kin_main (int argc, char *argv[]);
void relax_arm();
void arm_fetch();
void arm_drop();
void stand_arm();

#endif