#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lcmtypes/maebot_targeting_laser_command_t.h"

#define NUM_BLINKS 3

int
main (int argc, char *argv[])
{
  lcm_t *lcm = lcm_create (NULL);
  maebot_targeting_laser_command_t msg;

  msg.laser_power = 1;
  maebot_targeting_laser_command_t_publish (lcm, "MAEBOT_TARGETING_LASER_COMMAND", &msg);
  usleep (250000);

  for (int i = 1; i < NUM_BLINKS; i++) {
      msg.laser_power = 0;
      maebot_targeting_laser_command_t_publish (lcm, "MAEBOT_TARGETING_LASER_COMMAND", &msg);
      usleep (250000);

      msg.laser_power = 1;
      maebot_targeting_laser_command_t_publish (lcm, "MAEBOT_TARGETING_LASER_COMMAND", &msg);
      usleep (250000);
    }

  msg.laser_power = 0;
  maebot_targeting_laser_command_t_publish (lcm, "MAEBOT_TARGETING_LASER_COMMAND", &msg);

  return 0;
}
