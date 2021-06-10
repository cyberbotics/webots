/*
 * Description:  Test Supervisor device AP: wb_supervisor_node_set_joint_position
 *                 
 */
 
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  ts_setup(argv[0]);
  
  WbNodeRef ball_joint_node = wb_supervisor_node_get_from_def("BALL_JOINT");
  wb_supervisor_node_set_joint_position(ball_joint_node, 0.6, 3);
  
  
  WbNodeRef slider_joint_node = wb_supervisor_node_get_from_def("SLIDER_JOINT");
  wb_supervisor_node_set_joint_position(slider_joint_node, 0.6, 1);
  
  WbNodeRef hinge_joint_node = wb_supervisor_node_get_from_def("HINGE_JOINT");
  wb_supervisor_node_set_joint_position(hinge_joint_node, 0.6, 1);
  
  WbNodeRef hinge2_joint_node = wb_supervisor_node_get_from_def("HINGE2_JOINT");
  wb_supervisor_node_set_joint_position(hinge2_joint_node, 0.6, 2);
  
  
  WbDeviceTag shoulder_lift_motor = wb_robot_get_device("shoulder_lift_joint");
  WbNodeRef shoulder_lift_motor_node = wb_supervisor_node_get_from_device(shoulder_lift_motor);
  WbNodeRef shoulder_lift_joint_node = wb_supervisor_node_get_parent_node(shoulder_lift_motor_node);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, -1.55, 1);
  
  
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, 0, 2);
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, 0, 0);
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, 0, -1);
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, 0, 1);
  wb_robot_step(TIME_STEP);

  
  
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };


  ts_send_success();
  return EXIT_SUCCESS;

  return 0;
}
