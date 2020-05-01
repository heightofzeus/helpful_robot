/*
Author: Dhruv Sharma
        ds1
        01 May 2020

16-311 Helpful Robot Project Code

Following is a controller for Pioneer 3-DX, which allows it to reach certain 
locations, while avoiding obstacles. 

Partially code is adapted from Tele-Operation Lab

*/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/camera.h>


#define MAX_SPEED 5.24
#define MAX_SENSOR_NUMBER 16
#define DELAY 70
#define MAX_SENSOR_VALUE 1024

// minimal distance, in meters, for an obstacle to be considered
#define MIN_DISTANCE 0.5

// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 100

typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

typedef enum { FORWARD, LEFT, RIGHT } State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag left_wheel = wb_robot_get_device("left wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel");

  WbDeviceTag red_led[3];
  red_led[0] = wb_robot_get_device("red led 1");
  red_led[1] = wb_robot_get_device("red led 2");
  red_led[2] = wb_robot_get_device("red led 3");

  char sensor_name[5] = "";
  int i;

  for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
    sprintf(sensor_name, "so%d", i);
    sensors[i].device_tag = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i].device_tag, time_step);
  }

  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  int j, led_number = 0, delay = 0;
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;

  // initial state
  State state = FORWARD;

  // House 1
  double home1[3] = { 4.54, 2.6, 1.87 };
  double home2[3] = { 6.22, 2.6, 3.22 };
  double home3[3] = { 8.5, 2.6, 2.75  };
  double home4[3] = { 12.1, 2.6, 4.22 };
  double home5[3] = { 7.41, 2.6, 7.88 };

  // Enable camera to click picure at checkpoint. 
  WbDeviceTag cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, time_step);

  // Enable gps device, that gives absolute location
  WbDeviceTag gps_sensor = wb_robot_get_device("gps");
  wb_gps_enable(gps_sensor, time_step);

  bool one = false;
  bool two = false;
  bool three = false;
  bool four = false;
  bool five = false;
  bool end = false;
  bool start = true;

  // start location
  const double *startLoc = malloc(sizeof(double)*3);
  double xS; double yS; double zS; 

  // run simulation
  while (wb_robot_step(time_step) != -1) {
    // initialize speed and wheel_weight_total arrays at the beginning of the loop
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);

    if (start) {
      // finding start location values
      startLoc = wb_gps_get_values(gps_sensor);
      xS = startLoc[0]; yS = startLoc[1]; zS = startLoc[2];
      start = false;
      printf("Starting coordinates: ");
      printf("xS = %f, yS = %f, zS = %f\n", xS, yS, zS);
    }

    // Figure out robot location
    const double *currLoc = malloc(sizeof(double)*3);
    currLoc = wb_gps_get_values(gps_sensor);
    double x = currLoc[0]; double y = currLoc[1]; double z = currLoc[2];

    // check if locations have been reached 
    if (fabs(x - home1[0]) < 0.5 && fabs(y - home1[1]) < 0.2 && 
                                fabs(z - home1[2]) < 0.5)
      {
        if (!one) {
            // mark that home1 has been reached
            printf("House 1 reached at: ");
            printf("x = %f, y = %f, z = %f\n", x, y, z);
            one = true;
        }
      }

    if (fabs(x - home2[0]) < 0.5 && fabs(y - home2[1]) < 0.2 && 
                                fabs(z - home2[2]) < 0.5)
      {
        if (one && !two) {
            // mark that home1 has been reached
            printf("House 2 reached at: ");
            printf("x = %f, y = %f, z = %f\n", x, y, z);
            two = true;
        }
      }    

    if (fabs(x - home3[0]) < 0.5 && fabs(y - home3[1]) < 0.2 && 
                                fabs(z - home3[2]) < 0.5)
      {
        if (two && !three) {
            // mark that home1 has been reached
            printf("House 3 reached at: ");
            printf("x = %f, y = %f, z = %f\n", x, y, z);
            three = true;
        }
      }  

    if (fabs(x - home4[0]) < 0.5 && fabs(y - home4[1]) < 0.2 && 
                                fabs(z - home4[2]) < 0.5)
      {
        if (three && !four) {
            // mark that home1 has been reached
            printf("House 4 reached at: ");
            printf("x = %f, y = %f, z = %f\n", x, y, z);
            four = true;
        }
      }  


    if (fabs(x - home5[0]) < 0.5 && fabs(y - home5[1]) < 0.2 && 
                                fabs(z - home5[2]) < 0.5)
      {
        if (four && !five) {
            // mark that home1 has been reached
            printf("House 5 reached at: ");
            printf("x = %f, y = %f, z = %f\n", x, y, z);
            five = true;
        }
      }  

    // check if start point is reached
    if (fabs(x - xS) < 0.5 && fabs(y - yS) < 0.2 && fabs(z - zS) < 0.5)
      {

        if (five && !end) {
          // we need to stop at this point, so the loop must end
          end = true;
          wb_motor_set_velocity(left_wheel, 0);
          wb_motor_set_velocity(right_wheel, 0);
          printf("Reached end of loop: ");
          printf("x = %f, y = %f, z = %f\n", x, y, z);
          return 0;
        }
      }


    for (i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      sensor_value = wb_distance_sensor_get_value(sensors[i].device_tag);

      if (sensor_value == 0.0)
        speed_modifier = 0.0;
      else {
        // actual distance to the obstacle, based on sensor value
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  

        // if the obstacle is close enough, we may want to turn
        if (distance < MIN_DISTANCE)
          speed_modifier = 1 - (distance / MIN_DISTANCE);
        else
          speed_modifier = 0.0;
      }

      // add the modifier for both wheels
      for (j = 0; j < 2; ++j)
        wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
    }

    switch (state) {
      // start turning in either direction when an obstacle is close enough
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      // go on in the same direction until no more obstacle are in sight
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
        free((void*)currLoc);
    }

    ++delay;
    if (delay == DELAY) {
      wb_led_set(red_led[led_number], 0);
      ++led_number;
      led_number = led_number % 3;
      wb_led_set(red_led[led_number], 1);
      delay = 0;
    }

    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
  }

  wb_robot_cleanup();
  free((void*)startLoc);
  

  return 0;
}