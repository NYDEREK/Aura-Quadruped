#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "robot_model.h"

int main(void)
{
    robot_model_t robot;
    robot_model_init(&robot);
    assert(!robot_model_is_complete(&robot));
    assert(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].config.servo_id ==
           ROBOT_SERVO_ID_UNASSIGNED);

    robot_axis_config_t hip = {
        .servo_id = 11, .direction = -1, .center_tick = 2048,
        .minimum_cdeg = -6000, .maximum_cdeg = 7500,
        .maximum_speed_raw = 500, .acceleration = 32,
    };
    assert(robot_model_assign(&robot, ROBOT_LEG_LEFT_FRONT, ROBOT_AXIS_HIP, &hip));
    assert(!robot_model_assign(&robot, ROBOT_LEG_RIGHT_FRONT, ROBOT_AXIS_HIP, &hip));
    assert(robot_model_set_target(&robot, ROBOT_LEG_LEFT_FRONT, ROBOT_AXIS_HIP,
                                  2.0f, 700, 40));
    const robot_axis_state_t *axis = &robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP];
    int16_t position;
    uint16_t speed;
    uint8_t acceleration;
    assert(robot_model_encode_target(axis, &position, &speed, &acceleration));
    // 2 radians exceeds 75°, so the software limit must be applied before
    // conversion and direction inversion.
    assert(position < 2048 && speed == 700 && acceleration == 40);

    uint8_t feedback[15] = {0};
    feedback[0] = 0; feedback[1] = 8;   // 2048 ticks, physical zero
    feedback[2] = 7; feedback[3] = 0x80; // -7 speed units
    feedback[4] = 3; feedback[5] = 0x04; // -3 load units
    feedback[6] = 120; feedback[7] = 41; feedback[10] = 1;
    feedback[13] = 9; feedback[14] = 0x80; // -9 current units
    robot_model_update_feedback(&robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP], feedback);
    assert(fabsf(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].measured_radians) < 0.001f);
    assert(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].measured_speed_raw == -7);
    assert(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].measured_load_raw == -3);
    assert(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].measured_current_raw == -9);
    assert(robot.axis[ROBOT_LEG_LEFT_FRONT][ROBOT_AXIS_HIP].present);
    float frame[4][3]={{0}};
    frame[0][1]=0.25f; frame[3][2]=NAN;
    const robot_model_t before=robot;
    assert(!robot_model_set_frame(&robot,frame,3400,254));
    assert(memcmp(&robot,&before,sizeof(robot))==0); // even last-axis failure is atomic
    frame[3][2]=0;
    assert(robot_model_set_frame(&robot,frame,3400,254));
    assert(fabsf(robot.axis[0][1].target_radians-0.25f)<1e-6f);
    assert(robot.axis[0][1].config.center_tick==before.axis[0][1].config.center_tick);
    assert(robot.axis[0][1].config.direction==before.axis[0][1].config.direction);
    assert(robot.axis[0][1].config.minimum_cdeg==before.axis[0][1].config.minimum_cdeg);
    assert(robot.axis[0][1].config.maximum_cdeg==before.axis[0][1].config.maximum_cdeg);
    puts("robot model tests passed");
    return 0;
}
