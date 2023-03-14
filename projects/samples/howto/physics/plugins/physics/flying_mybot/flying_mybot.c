/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of use of a custom ODE physics
 *               plugin.
 */

#include <plugins/physics.h>
#include <stdlib.h>

#define FLYING_FORCE 6.867
#define FLYING_INCREMENT 0.3
#define TURNING_INCREMENT 0.01
#define RAY_LENGTH 0.4

typedef enum { TAKING_OFF, SEARCH_OBSTACLE, MOVE_UP, AVOID_OBSTACLE, MOVE_DOWN, LANDING } states;
typedef enum { LEFT, RIGHT } turning_directions;

static pthread_mutex_t mutex;

/* The Geoms used in our plugin. */
static dGeomID robot_geom;
static dGeomID ray_geom;
static dGeomID floor_geom;
static dGeomID wings_geom[2];

/* The Bodies used in our plugin. */
static dBodyID robot_body;
static dBodyID wings_body[2];

static states state;
static turning_directions turning_direction;
static int obstacle_detected;
static int ray_collision_occured;
static int rotating_iterations;
static int inversing_direction_counter;
static int pause_counter;

/*
 * This function is called every time ODE is started, this is a good place to
 * initialize the variables and to create the objects we want. It is also
 * useful to store the general ODE variables.
 */
void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);  // needed to run with multi-threaded version of ODE

  /*
   * Here we get all the geoms associated with DEFs in Webots.
   * A Geom corresponds to the boundingObject node of the object specified
   * by the DEF name.
   * That is why you can retreive here only DEFs of nodes which contains a
   * boundingObject node.
   */
  robot_geom = dWebotsGetGeomFromDEF("ROBOT");
  floor_geom = dWebotsGetGeomFromDEF("FLOOR");
  wings_geom[0] = dWebotsGetGeomFromDEF("LEFT_WING");
  wings_geom[1] = dWebotsGetGeomFromDEF("RIGHT_WING");

  /*
   * Then using these Geoms we can get the bodies.
   * A Body corresponds to the physics node of the object which contains the
   * boundingObject corresponding to the Geom.
   * In this particular case, we need to use the dSpaceGetGeom(geom, 0)
   * function as the Geom of the robot is in fact a Space. As all the Geoms
   * of the Space have the same Body, we use the argument 0 to use
   * always the first Geom to get the Body.
   * You retreive a Space instead of a Geom when there are various shapes in
   * the same boundingObject node. If you no not know wheter the geom is a
   * space or not, you can use the dGeomIsSpace(geom) function.
   */
  if (robot_geom)
    robot_body = dGeomGetBody(dSpaceGetGeom((dSpaceID)robot_geom, 0));
  dWebotsConsolePrintf("Initialization: ROBOT = %p %p\n", robot_geom, robot_body);

  /* We create a Ray object */
  ray_geom = dCreateRay(dGeomGetSpace(robot_geom), RAY_LENGTH);
  dGeomSetDynamicFlag(ray_geom);

  if (wings_geom[0] && wings_geom[1]) {
    wings_body[0] = dGeomGetBody(wings_geom[0]);
    wings_body[1] = dGeomGetBody(wings_geom[1]);
  }

  state = LANDING;
  turning_direction = LEFT;
  pause_counter = 10;

  rotating_iterations = 0;

  obstacle_detected = 0;
  ray_collision_occured = 0;
}

/*
 * This function is called at every step of ODE. This is the good place to add
 * forces on the robot or the move some objects in the world.
 */
void webots_physics_step() {
  const dReal *position = dBodyGetPosition(robot_body);
  const dReal *rotation = dBodyGetRotation(robot_body);

  /* We receive the sensors values from the controller. */
  int size;
  const float *sensors = dWebotsReceive(&size);

  float flying_force = FLYING_FORCE;
  float turning = TURNING_INCREMENT;

  if (turning_direction == LEFT)
    turning *= -1;

  states next_state = state;

  /*
   * We use a small finite state machine to move our robot. The main idea is
   * quite simple :
   *  1. We are on the ground, we choose in which direction we will turn and
   *     how many obstacles we will avoid.
   *  2. We move our robot up in the air at a defined altitude.
   *  3. At this altitude we turn on a given direction until the sensor
   *     detects an obstacle.
   *  4. We stop turning and move up until we are upper then the
   *     obstacle
   *  5. We turn again until our ray has passed the obstacle
   *  6. After that, we stop turning and we move down to the previous
   *     altitude.
   *  7. If we have avoided enough obstacles, we land. If not we continue
   *     like this.
   */
  switch (state) {
    case LANDING:
      turning = 0;

      /* We must make a sort pause to stabilize the robot on the ground. */
      if (pause_counter == 0) {
        pause_counter = 20;

        /*
         * We choose randomly the number of obstacles to avoid and we
         * invert the movement direction.
         */
        inversing_direction_counter = (int)(6 * ((float)rand() / RAND_MAX) + 1);
        turning_direction = (turning_direction == LEFT) ? RIGHT : LEFT;

        /* Send a message to the controller with new data */
        int buffer[2];
        buffer[0] = inversing_direction_counter;
        buffer[1] = (turning_direction == LEFT) ? 0 : 1;
        dWebotsSend(1, buffer, sizeof(buffer));

        /* We give an impulse to move the robot up. */
        flying_force += FLYING_INCREMENT;
        next_state = TAKING_OFF;
      } else {
        flying_force = 6;

        if (position[2] < 0.001)
          pause_counter--;
      }
      break;
    case TAKING_OFF:
      turning = 0;

      if (position[2] > 0.01) {
        /* We give an opposite impulse to stop the robot. */
        flying_force -= FLYING_INCREMENT;
        next_state = SEARCH_OBSTACLE;
      }
      break;
    case SEARCH_OBSTACLE:

      /*
       * We use only the sensor on the side corresponding to the rotation
       * direction to detect the obstacle.
       */
      if (sensors && sensors[turning_direction] != 0) {
        next_state = MOVE_UP;
        flying_force += FLYING_INCREMENT;

        /* We compensate the previous forces to stop our robot. */
        turning *= -rotating_iterations;
        rotating_iterations = 0;
      } else
        rotating_iterations++;
      break;
    case MOVE_UP:
      turning = 0;
      if (sensors && sensors[turning_direction] == 0) {
        /* This compensate the up movement of the robot. */
        flying_force -= FLYING_INCREMENT;
        next_state = AVOID_OBSTACLE;
      }
      break;
    case AVOID_OBSTACLE:
      if (ray_collision_occured == 0 && obstacle_detected == 1) {
        obstacle_detected = 0;
        turning *= -rotating_iterations / 10;
        rotating_iterations = 0;
        next_state = MOVE_DOWN;
        flying_force -= FLYING_INCREMENT;
      } else {
        if (ray_collision_occured > 0)
          obstacle_detected = 1;
        turning /= 10;
        rotating_iterations++;
      }
      break;
    case MOVE_DOWN:
      if (position[2] <= 0.01) {
        /* We have avoided one more obstacle. */
        inversing_direction_counter--;
        if (inversing_direction_counter == 0) {
          next_state = LANDING;
          flying_force = 6;
        } else {
          next_state = SEARCH_OBSTACLE;
          flying_force += FLYING_INCREMENT;
        }
      }
      turning = 0;
      break;
  }

  /* We update the state of our automata. */
  state = next_state;

  /* We add the forces to our robot. */
  dBodyAddRelForce(wings_body[0], turning, 0, flying_force / 2);
  dBodyAddRelForce(wings_body[1], -turning, 0, flying_force / 2);

  /* We place the ray in front of the robot. */
  dGeomRaySet(ray_geom, position[0], position[1], position[2], rotation[0], -rotation[1], rotation[2]);

  /*
   * We reset the ray collision detection one iteration after
   * (to leave the time to display it).
   */
  if (ray_collision_occured > 0)
    ray_collision_occured--;
}

/*
 * This function is called every time a collision is detected between two
 * Geoms. It allows you to handle the collisions as you want.
 * Here you can either simply detect collisions for informations, disable
 * useless collisions or handle them.
 * This function is called various times for each time step.
 * For a given collision, if you return 1 this means that you have handled the
 * collision yourself and so it will be ignored by Webots. If you return 0,
 * this means that you want Webots to handle it for you.
 * The g1 and g2 parameter may refer only to placeable ODE Geoms (no Space).
 * To be able to identify who these Geoms are, you must compare them with the
 * ones you have stored. If the Geom you have stored is really a Geom too,
 * you can compare them simply using the == operator but if it is a Space, you
 * should use the dSpaceQuery(geom,gX) function.
 */
int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * For the collisions, we allow up to 10 contact points,
   * this is probably overkilled.
   */
  dContact contact[10];
  dVector3 start, d;

  int i, n, ray_color;

  /* First we check if the collision is involving the Ray. */
  if (dAreGeomsSame(g1, ray_geom) || dAreGeomsSame(g2, ray_geom)) {
    /*
     * But in the case the collision is with the robot's body, we ignore
     * it. They will not create any collision.
     */
    if ((dAreGeomsSame(g2, ray_geom) && (dSpaceQuery((dSpaceID)robot_geom, g1) == 1 || dGeomIsSpace(g1))) ||
        (dAreGeomsSame(g1, ray_geom) && (dSpaceQuery((dSpaceID)robot_geom, g2) == 1 || dGeomIsSpace(g1)))) {
      ray_color = 0;
      dWebotsSend(2, &ray_color, sizeof(int));
      return 1;
    }

    /*
     * If not, we create the contact points and we use the first one to
     * compute the distance from the robot to the obstacle.
     */
    n = dCollide(g1, g2, 10, &contact[0].geom, sizeof(dContact));
    if (n == 0) {
      ray_color = 0;
      dWebotsSend(2, &ray_color, sizeof(int));
      return 1;
    }

    dVector3 dir;
    dGeomRayGet(ray_geom, start, dir);
    for (i = 0; i < 3; i++)
      d[i] = start[i] - contact[0].geom.pos[i];
    const float ray_distance = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

    dWebotsConsolePrintf("Obstacle detected at a distance of %g after %g s\n", ray_distance, dWebotsGetTime() / 1000);

    ray_color = ray_collision_occured = 2;

    /* Send a message to the controller to update the ray color */
    dWebotsSend(2, &ray_color, sizeof(int));

    return 2;  // tell Webots to notify the collision Ray vs Box by a change in the boundingObject color

    /* Here we disable all the possible collisions with our wings. */
  } else if ((dAreGeomsSame(g1, wings_geom[0]) || dAreGeomsSame(g1, wings_geom[1])) ||
             (dAreGeomsSame(g2, wings_geom[0]) || dAreGeomsSame(g2, wings_geom[1]))) {
    return 1;

    /*
     * Here we handle the collision between the floor and our robot in our
     * particular way. In order to do that, we first need to create the
     * collision points and then we modify the parameters of the contact
     * surface.
     */
  } else if ((dAreGeomsSame(g1, floor_geom) && dSpaceQuery((dSpaceID)robot_geom, g2) == 1) ||
             (dSpaceQuery((dSpaceID)robot_geom, g1) == 1 && dAreGeomsSame(g2, floor_geom))) {
    n = dCollide(g1, g2, 10, &contact[0].geom, sizeof(dContact));
    if (n == 0)
      return 1;
    dBodyID body = dGeomGetBody(g1);
    if (body == NULL)
      body = dGeomGetBody(g2);
    if (body == NULL)
      return 0;
    dWorldID world = dBodyGetWorld(body);
    dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();

    /*
     * In our particular case, we want a collision which is totally not
     * bouncy. For more information on the possible parameters, please
     * refer to the ODE documenatation.
     */
    for (i = 0; i < n; i++) {
      contact[i].surface.mode = dContactMu2 | dContactBounce | dContactApprox1 | dContactSoftCFM;
      contact[i].surface.mu = 0.4;
      contact[i].surface.mu2 = 0.8;
      contact[i].surface.bounce = 0;
      contact[i].surface.bounce_vel = 0;
      contact[i].surface.soft_cfm = 0.001;

      /* We add these points to the simulation. */
      pthread_mutex_lock(&mutex);
      dJointAttach(dJointCreateContact(world, contact_joint_group, &contact[i]), robot_body, NULL);
      pthread_mutex_unlock(&mutex);
    }
    return 1;

    /* We have not handled the collision. */
  } else
    return 0;
}

/*
 * This function is called every time the World is destroyed or restarted. It
 * is mandatory to destroy here everything you have created. If you do not do
 * it, the stability of Webots cannot be guaranteed.
 */
void webots_physics_cleanup() {
  dGeomDestroy(ray_geom);
  pthread_mutex_destroy(&mutex);
  dWebotsConsolePrintf("Clean up\n");
}
