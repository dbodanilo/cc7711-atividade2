/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28

#ifndef PI
#define PI 3.14159
#endif

#define NUM_SENSORS 8
#define NUM_BOXES 9
#define MAP_LENGTH 20

static double weights[NUM_SENSORS][2] = {
  {-1.3,  -1.0}, 
  {-1.3,  -1.0}, 
  {-0.5,   0.5},
  { 0.0,   0.0},
  { 0.0,   0.0}, 
  { 0.05, -0.5},
  {-0.75,  0}, 
  {-0.75,  0}
};

static double offsets[2] = {0.7 * MAX_SPEED, 0.7 * MAX_SPEED};

typedef struct {
  WbFieldRef ref;
  // union type, if other value type comes up
  double const* value;
} Field;

typedef struct {
  WbDeviceTag tag;
  double value;
} Sensor;

typedef struct {
  char *name;
  WbNodeRef node;
  Field translation;
  bool visited;
} Box;

typedef struct {
  Box *target;

} Map;

typedef struct {
  char const* name;
  WbDeviceTag motors[2];
  double velocities[2];

  WbNodeRef node;
  Field translation;
  Field rotation;
  Sensor sensors[NUM_SENSORS];
} Robot;

static int get_time_step() {
  static int time_step = -1;

  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();

  return time_step;
}

double get_sign(double x) {
  return x/fabs(x);
}

double get_distance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool map_from_world(int *m, double w) {
  *m = (int)floor(20.0 * w) + 10;

  return *m >= 0 && *m < MAP_LENGTH;
} 

bool world_from_map(double *w, int m) {
  *w = (m - 10.0) / 20.0;  

  return *w >= -0.5 && *w <= 0.5;
}

void robot_update(Robot *r) {
  r->translation.value = 
      wb_supervisor_field_get_sf_vec3f(r->translation.ref);
  r->rotation.value = 
      wb_supervisor_field_get_sf_rotation(r->rotation.ref);
  
  for (int i = 0; i < NUM_SENSORS; i += 1) {
    r->sensors[i].value = 
      wb_distance_sensor_get_value(r->sensors[i].tag);
  }

//  r->velocities[LEFT] = "?"; 
//  r->velocities[RIGHT] = "?";
}

void robot_actuate(Robot const* r) {
  wb_motor_set_velocity(r->motors[LEFT], r->velocities[LEFT]);
  wb_motor_set_velocity(r->motors[RIGHT], r->velocities[RIGHT]);
}

bool robot_init(Robot *r) {
  r->name = "e-Puck";

  r->node = wb_supervisor_node_get_from_def(r->name);
  if (r->node == NULL) {
    fprintf(stderr, "No DEF %s node found in the current world file\n", r->name);
    return false;
//    exit(1);
  }

  r->translation.ref = wb_supervisor_node_get_field(r->node, "translation");
  r->rotation.ref = wb_supervisor_node_get_field(r->node, "rotation");

  /* You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  // MOTORS
  r->motors[LEFT] = wb_robot_get_device("left wheel motor");
  r->motors[RIGHT] = wb_robot_get_device("right wheel motor");

  // SENSORS
  for (int i = 0; i < NUM_SENSORS; i += 1) {
    char *name = (char *)malloc(sizeof(char)*4);
    sprintf(name, "ps%d", i);

    r->sensors[i].tag = wb_robot_get_device(name);
    
    wb_distance_sensor_enable(r->sensors[i].tag, get_time_step());

    free(name);
  }

  // INTIAL VALUES
  wb_motor_set_position(r->motors[LEFT], INFINITY);
  wb_motor_set_position(r->motors[RIGHT], INFINITY);

  // starting speed, before setting target box
  r->velocities[LEFT]  = 0.5 * MAX_SPEED;
  r->velocities[RIGHT] = 0.5 * MAX_SPEED;

  robot_update(r);
  robot_actuate(r);
 
  return true;
}

void box_update(Box* b) {
  b->translation.value = 
      wb_supervisor_field_get_sf_vec3f(b->translation.ref);
}

bool box_init(Box* b, int i) {
  // "Caixa%d"
  b->name = (char *)malloc(sizeof(char)*7);
  sprintf(b->name, "Caixa%d", i);

  b->node = wb_supervisor_node_get_from_def(b->name);
  if (b->node == NULL) {
    fprintf(stderr, 
      "No DEF %s node found in the current world file\n", 
      b->name);
    return false;
//    exit(1);
  }
  
  b->translation.ref = 
      wb_supervisor_node_get_field(b->node, "translation");
 
  b->visited = false;

  box_update(b);

  return true;
}

void box_free(Box *b) {
  free(b->name);
}

bool robot_rotate_to(Robot *r, double to) {
  // allowed error (rad)
  double e = 0.1;
  double angle = r->rotation.value[1] * r->rotation.value[3];

  double diff = to - angle;
  if (diff > PI) diff -= PI;

  while (fabs(diff) > e) {
    if (diff < 0) {
      r->velocities[LEFT]  =  0.25 * MAX_SPEED;
      r->velocities[RIGHT] = -0.25 * MAX_SPEED;
    }
    else {
      r->velocities[LEFT]  = -0.25 * MAX_SPEED;
      r->velocities[RIGHT] =  0.25 * MAX_SPEED;
    }

    robot_actuate(r);

    if (wb_robot_step(TIME_STEP) == -1) return false;

    robot_update(r);
    angle = r->rotation.value[1] * r->rotation.value[3];

    diff = to - angle;
    if (diff > PI) diff -= PI;

    printf("angle: %g\n", angle);
    printf("   to: %g\n", to);
    printf(" diff: %g\n", diff);
  } 

  return true;
}

// 0.0 as a positive coordinate
int get_quadrant(double x, double y) {
  int q = 1; 

  if (x < 0) {
    if (y < 0) q = 3;
    // y >= 0
    else q = 2;
  }
  // x >= 0
  else if (y < 0) q = 4;

  return q;
}

double angle_from_position(double x, double y) {
  double dist = get_distance(x, y, 0.0, 0.0);

  int q = get_quadrant(x, y);

  double arc = acos(x/dist);
  double angle = arc;

  switch(q) {
    case 1: angle =  arc - PI/2.0; break;
    case 2: angle =  arc - PI/2.0; break;
    case 3: angle =  PI - (arc - PI/2.0); break;
    case 4: angle = -arc - PI/2.0; break;
    default: break; // unreachable
  }

  return angle;
}

bool get_next_position(int *x, int *y, 
                       int m[MAP_LENGTH][MAP_LENGTH],
                       int rx, int ry) {
//  int rx, ry;
//  map_from_world(&rx, r->translation.value[0]);
//  map_from_world(&ry, r->translation.value[2]);

  int min_ij = m[rx][ry];

  for (int i = -1; i < 2; i += 1) {
    for (int j = -1; j < 2; j += 1) {
      int xi = rx + i;
      int yj = ry + j;

      if ((xi >= 0 && xi < MAP_LENGTH)
          && (yj >= 0 && yj < MAP_LENGTH)) {
        if (m[xi][yj] != 1 && m[xi][yj] < min_ij) {
          min_ij = m[xi][yj];

          *x = xi;
          *y = yj;
        }
      }
    }
  }
  
  return min_ij < m[rx][ry];
}

void map_adjacency(int m[MAP_LENGTH][MAP_LENGTH], 
                   int x, int y) {
  bool altered[3][3] = { false };

  for (int i = -1; i < 2; i += 1) {
    for (int j = -1; j < 2; j += 1) {
      int xi = x + i;
      int yj = y + j;

      if ((xi >= 0 && xi < MAP_LENGTH)
          && (yj >= 0 && yj < MAP_LENGTH)) {
        if (m[xi][yj] == 0) {
          m[xi][yj] = m[x][y] + 1;

          altered[i + 1][j + 1] = true;
        }
      }
    }
  }

  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < 3; j += 1) {
      if (altered[i][j]) map_adjacency(m, x + i - 1, y + j - 1);
    }
  }
}

void print_map(int m[MAP_LENGTH][MAP_LENGTH]) {
  for (int i = 0; i < MAP_LENGTH; i += 1) {
    for (int j = 0; j < MAP_LENGTH; j += 1) {
      printf("%4d", m[i][j]);
    }
    printf("\n");
  }
}

void map_target(int m[MAP_LENGTH][MAP_LENGTH], 
                Robot *r, Box *bs, Box *t) {
  for (int i = 0; i < MAP_LENGTH; i += 1) {
    for (int j = 0; j < MAP_LENGTH; j += 1) {
      m[i][j] = 0;
    }
  }

  for (int i = 0; i < NUM_BOXES; i += 1) {
    Box *b = bs + i;

    int bx, by;
    map_from_world(&bx, b->translation.value[0]);
    map_from_world(&by, b->translation.value[2]);
    
    m[bx][by] = 1;
  }

  printf("map: \n");
  print_map(m);
  printf("\n");

  int rx, ry;
  map_from_world(&rx, r->translation.value[0]);
  map_from_world(&ry, r->translation.value[2]);

  int tx, ty;
  map_from_world(&tx, t->translation.value[0]);
  map_from_world(&ty, t->translation.value[2]);

  m[rx][ry] = 0;
  m[tx][ty] = 2;

  map_adjacency(m, tx, ty);

  printf("map: \n");
  print_map(m);
  printf("\n");
}

Box *get_target(Robot *r, Box *bs) {
  Box *target = NULL;

  for (int i = 0; i < NUM_BOXES; i += 1) {
    Box *b = bs + i;
   
    if (!b->visited) {
      double dist = 
        get_distance(r->translation.value[0], 
                     r->translation.value[2], 
                     b->translation.value[0], 
                     b->translation.value[2]);
//      printf("distance: %.2lf\n", dist);
      if (target == NULL) {
        // set target
        target = b;
        break;
      }
    }
  }

  return target;
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // ARENA MAP
//  bool visited_boxes[NUM_BOXES] = { false };

  Robot epuck;
  if (!robot_init(&epuck)) exit(1);

  // POSIÇÃO ROBO
  // ATENÇÃO -> NÃO ESQUEÇA DE HABILITAR O SUPERVISOR 
  // *E* NOMEAR (DEF) O ROBO!
  Box boxes[NUM_BOXES];
  int arena[MAP_LENGTH][MAP_LENGTH] = { 0 };
  
  for (int i = 0; i < NUM_BOXES; i += 1) {
    Box *b = boxes + i;
    box_init(b, i);
    
    printf("Posição %s: %6.2f  %6.2f\n", 
           b->name, 
           b->translation.value[0], 
           b->translation.value[2]);

//    int x, y;
//    indexes_from_translation(&x, &y, 
//      b->translation.value[0], b->translation.value[2]);
  }

//  bool rotated = false; 
  Box* target = NULL;
  int old_x = -1;
  int old_y = -1;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    // updated translation & rotation
    robot_update(&epuck);
  
    // check visited
    double limit = sqrt(0.02)/2.0 + 0.074/2.0;
    printf("limit: %.2lf\n", limit);
    for (int i = 0; i < NUM_BOXES; i += 1) {
      Box *b = boxes + i;

      if (!b->visited) {
        double dist = 
          get_distance(epuck.translation.value[0], 
                       epuck.translation.value[2], 
                       b->translation.value[0], 
                       b->translation.value[2]);

        printf("distance: %.2lf\n", dist);

        if (dist < limit) {
          b->visited = true;
        }
      } else printf("\n%s has been visited!\n\n", b->name);
    }

    Box *old_target = target;

    // follow boxes, avoid walls
    if (target == NULL || target->visited) target = get_target(&epuck, boxes);
  
    if (target != old_target) map_target(arena, &epuck, boxes, target);

    printf("arena: \n");
    print_map(arena);
    printf("\n");

    double rwx = epuck.translation.value[0];
    double rwy = epuck.translation.value[2];

    int rx, ry;
    map_from_world(&rx, rwx);
    map_from_world(&ry, rwy);

    printf("current position: %2d, %2d\n", rx, ry);

    if (rx != old_x && ry != old_y) {
      old_x = rx;
      old_y = ry;

      int x, y;
      get_next_position(&x, &y, arena, rx, ry);

      printf("next position: %2d, %2d\n", x, y);

      double angle = 0.0;

      if (x < rx) {
        if (y < ry)      angle =  3.0 * PI/4.0;
        else if (y > ry) angle = -3.0 * PI/4.0;
        // y == ry
        else             angle = PI; 
        
      } else if (x > rx) {
        if (y < ry)      angle =  PI/4.0;
        else if (y > ry) angle = -PI/4.0;
        // y == ry
        else             angle = 0.0;
 
      }
      // rx == x  
      else {
        if (y < ry)      angle =  PI/2.0;
        else if (y > ry) angle = -PI/2.0;
        // y == ry
        else             angle = 0.0;
      }

      double wx, wy;
      world_from_map(&wx, x);
      world_from_map(&wy, y);
 
//      double angle = angle_from_position(wx - rwx, wy - rwy);

      angle = -PI/2.0 + angle;
      if (fabs(angle) > PI) angle = get_sign(angle) * PI; 

      printf("angle: %.4f\n", angle);

      robot_rotate_to(&epuck, angle);
    }

//    printf("arena: \n");
//    print_map(arena); 

    // map environment
//    map_target(arena, target, &epuck);

    epuck.velocities[LEFT]  = 0.5 * MAX_SPEED;
    epuck.velocities[RIGHT] = 0.5 * MAX_SPEED;

    for (int i = 0; i < 2; i++) {
//       Velocidades[i] = 0.0;

       double sum = 0.0;
       double v_dev = 0.0;
   
       for (int j = 0; j < NUM_SENSORS; j++) {
         // weights[j][i]
         sum += fabs(epuck.sensors[j].value);
         v_dev += epuck.sensors[j].value 
                * weights[j][i] / 512.0;
       }
       
       epuck.velocities[i] += v_dev;
   
       printf("deviation  velocity %s: %.4lf (%.2lf %%)\n", 
              i == 0 ? "LEFT" : "RIGHT",
              v_dev, 100.0 * v_dev/sum);

//       epuck.velocities[i] *= 0.5*MAX_SPEED;

//       epuck.velocities[i] = (epuck.velocities[i] * MAX_SPEED) 
//                        + ((double)rand()/RAND_MAX-.5);

//       epuck.velocities[i] += ((double)rand()/RAND_MAX-.5);
   
       if (epuck.velocities[i] > MAX_SPEED)
         epuck.velocities[i] = MAX_SPEED;
       else if (epuck.velocities[i] < -MAX_SPEED)
         epuck.velocities[i] = -MAX_SPEED;
     }

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    robot_actuate(&epuck);


//    if (rotated) {
//      printf("\n\nrotacionado!\n\n");
//      rotated = false;
//    }

    printf("Velocidades: %6.2f %6.2f\n", 
           epuck.velocities[LEFT], epuck.velocities[RIGHT]);
    printf("Posição Robo: %6.2f  %6.2f\n", 
           epuck.translation.value[0], epuck.translation.value[2]); 
    printf("Rotação Robo: %6.2f\n", 
           epuck.rotation.value[1] * epuck.rotation.value[3]); 
  }

  /* Enter your cleanup code here */
  // free memory, I asssume
  for (int i = 0; i < NUM_BOXES; i += 1) box_free(boxes + i);

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

