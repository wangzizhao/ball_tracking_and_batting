#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"
#include "lcmtypes/dynamixel_config_list_t.h"
#include "lcmtypes/dynamixel_config_t.h"


#include "common/getopt.h"
#include "common/timestamp.h"
#include "math_util.h"

#include "dynamixel_device.h"
#include "dynamixel_serial_bus.h"

#define dmax(A, B) (A > B ? A : B)
#define dmin(A, B) (A < B ? A : B)
#define dabs(A) (A < 0 ? -A : A)

#define DYNAMIXEL_DRIVER_RATE_HZ 120

typedef struct arm_state arm_state_t;
struct arm_state
{
    // Arm communication
    dynamixel_bus_t *bus;
    dynamixel_device_t **servos;
    int num_servos;
    int msg_type;

    // LCM
    lcm_t *lcm;
    pthread_mutex_t cmd_lock;
    dynamixel_command_list_t *cmds;
    dynamixel_config_list_t *configs;
    const char *command_channel;
    const char *config_channel;
    const char *status_channel;

    // Threading
    pthread_mutex_t serial_lock;
    pthread_t status_thread;
    pthread_t driver_thread;
};

static arm_state_t *
arm_state_create (const char *busname, const int baud, const int initial_servo, const int num_servos)
{
    arm_state_t *arm_state = calloc (1, sizeof (*arm_state));
    arm_state->bus = serial_bus_create (busname, baud);
    arm_state->servos = calloc (num_servos, sizeof (*arm_state->servos));
    arm_state->num_servos = num_servos;
    arm_state->msg_type = 0;

    // Create the servos
    for (int id = 0; id < num_servos; id++) {
        arm_state->servos[id] = arm_state->bus->get_servo (arm_state->bus, id+initial_servo);
        if (arm_state->servos[id] != NULL) {
            printf ("Found %s servo id = %d\n",
                    arm_state->servos[id]->get_name (arm_state->servos[id]), id+initial_servo);
        }
        else {
            printf ("ERROR: Could not find servo id = %d, check power\n", id+initial_servo);
            exit (EXIT_FAILURE);
        }
    }

    return arm_state;
}

static void
arm_state_destroy (arm_state_t *arm_state)
{
    for (int i = 0; i < arm_state->num_servos; i++) {
        arm_state->servos[i]->destroy (arm_state->servos[i]);
    }
    arm_state->bus->destroy (arm_state->bus);
    free (arm_state->servos);
    lcm_destroy (arm_state->lcm);
    free (arm_state);
}


void *
status_loop (void *user)
{
    printf ("Starting status loop...\n");
    arm_state_t *arm_state = user;

    dynamixel_status_list_t stats;
    stats.len = arm_state->num_servos;
    stats.statuses = calloc (arm_state->num_servos, sizeof (*stats.statuses));
    while (1) {
        int64_t utime = utime_now ();

        pthread_mutex_lock (&arm_state->serial_lock);
        for (int id = 0; id < arm_state->num_servos; id++) {
            stats.statuses[id].utime = utime_now ();

            dynamixel_device_status_t *stat = arm_state->servos[id]->get_status(arm_state->servos[id]);

            stats.statuses[id].error_flags = stat->error_flags;
            stats.statuses[id].position_radians = stat->position_radians;
            stats.statuses[id].speed = stat->speed;
            stats.statuses[id].load = stat->load;
            stats.statuses[id].voltage = stat->voltage;
            stats.statuses[id].temperature = stat->temperature;

            dynamixel_device_status_destroy (stat);
        }
        pthread_mutex_unlock (&arm_state->serial_lock);

        // Publish
        dynamixel_status_list_t_publish (arm_state->lcm, arm_state->status_channel, &stats);

        // Attempt to send messages at a fixed rate
        int hz = DYNAMIXEL_DRIVER_RATE_HZ;
        int64_t max_delay = (1000000 / hz);
        int64_t now = utime_now ();
        int64_t delay = imin64 (now - utime, max_delay);
        utime = now;
        usleep (max_delay - delay);
    }

    return NULL;
}

// === LCM Handler ==============
static void
command_handler (const lcm_recv_buf_t *rbuf,
                 const char *channel,
                 const dynamixel_command_list_t *msg,
                 void *user)
{
    arm_state_t *arm_state = user;
    pthread_mutex_lock (&arm_state->cmd_lock);
    if (arm_state->cmds != NULL)
        dynamixel_command_list_t_destroy (arm_state->cmds);

    if (msg->len > arm_state->num_servos) {
        printf ("ERROR: cmds_len %d > num_servos %d, consuming only the first num_servos cmds\n",
                msg->len, arm_state->num_servos);

        int cmds_len = msg->len;
        dynamixel_command_list_t *msg_tmp = dynamixel_command_list_t_copy (msg);
        msg_tmp->len = arm_state->num_servos;
        arm_state->cmds = dynamixel_command_list_t_copy (msg_tmp);
        msg_tmp->len = cmds_len;
        dynamixel_command_list_t_destroy (msg_tmp);
    }
    else
        arm_state->cmds = dynamixel_command_list_t_copy (msg);

    arm_state->msg_type = 1;
    pthread_mutex_unlock (&arm_state->cmd_lock);
}

static void
config_handler (const lcm_recv_buf_t *rbuf,
                 const char *channel,
                 const dynamixel_config_list_t *msg,
                 void *user)
{
    arm_state_t *arm_state = user;
    pthread_mutex_lock (&arm_state->cmd_lock);
    if (arm_state->configs != NULL)
        dynamixel_config_list_t_destroy (arm_state->configs);

    if (msg->len > arm_state->num_servos) {
        printf ("ERROR: config_len %d > num_servos %d, consuming only the first num_servos cfgs\n",
                msg->len, arm_state->num_servos);

        int cmds_len = msg->len;
        dynamixel_config_list_t *msg_tmp = dynamixel_config_list_t_copy (msg);
        msg_tmp->len = arm_state->num_servos;
        arm_state->configs = dynamixel_config_list_t_copy (msg_tmp);
        msg_tmp->len = cmds_len;
        dynamixel_config_list_t_destroy (msg_tmp);
    }
    else
        arm_state->configs = dynamixel_config_list_t_copy (msg);
    arm_state->msg_type = 2;
    pthread_mutex_unlock (&arm_state->cmd_lock);
}

void *
driver_loop (void *user)
{
    printf ("Starting driver loop...\n");
    arm_state_t *arm_state = user;

    dynamixel_command_list_t last_cmds;
    last_cmds.len = arm_state->num_servos;
    last_cmds.commands = malloc (arm_state->num_servos * sizeof (dynamixel_command_t));
    for (int id = 0; id < arm_state->num_servos; id++) {
        last_cmds.commands[id].utime = 0;
        last_cmds.commands[id].position_radians = 0.0;
        last_cmds.commands[id].speed = 0.0;
        last_cmds.commands[id].max_torque = 0.0;
    }

    dynamixel_config_list_t last_cfgs;
    last_cfgs.len = arm_state->num_servos;
    last_cfgs.configs = malloc (arm_state->num_servos * sizeof (dynamixel_config_t));
        for (int id = 0; id < arm_state->num_servos; id++) {
        last_cfgs.configs[id].utime = 0;
        last_cfgs.configs[id].kp = 32;
        last_cfgs.configs[id].ki = 0;
        last_cfgs.configs[id].kd = 0;
        last_cfgs.configs[id].compl_margin = 1;
        last_cfgs.configs[id].compl_slope = 32;
        last_cfgs.configs[id].LED = 0;
    }


    // Handle messages as they come in from the arm
    while (1) {
        int hz = 100;
        int ret = lcm_handle_timeout (arm_state->lcm, 1000/hz);
        if (ret < 0) {
            printf ("ERROR: lcm_handle_timeout()\n");
            continue;
        }
        else if (ret == 0) // timeout
            continue;


        if(arm_state->msg_type == 1){
            // Get commands from somewhere
            pthread_mutex_lock (&arm_state->cmd_lock);
            dynamixel_command_list_t *cmds = arm_state->cmds;
            arm_state->msg_type = 0;
            pthread_mutex_unlock (&arm_state->cmd_lock);

            pthread_mutex_lock (&arm_state->serial_lock);
            for (int id = 0; id < cmds->len; id++) {
                dynamixel_command_t cmd = cmds->commands[id];
                dynamixel_command_t last_cmd = last_cmds.commands[id];

                int update_cmd = ((cmd.utime - last_cmd.utime) > 1000000 ||
                            last_cmd.position_radians != cmd.position_radians ||
                            last_cmd.speed != cmd.speed ||
                            last_cmd.max_torque != cmd.max_torque);

                if (update_cmd) {
                    arm_state->servos[id]->set_goal (arm_state->servos[id],
                                                 cmd.position_radians,
                                                 dmax(0.0, dmin(1.0, cmd.speed)),
                                                 dmax(0.0, dmin(1.0, cmd.max_torque)));
                    last_cmds.commands[id] = cmd;
                }
            }
        }

        if(arm_state->msg_type == 2){
            // Get configs
            pthread_mutex_lock (&arm_state->cmd_lock);
            dynamixel_config_list_t *cfgs = arm_state->configs;
            arm_state->msg_type = 0;
            pthread_mutex_unlock (&arm_state->cmd_lock);

            pthread_mutex_lock (&arm_state->serial_lock);
            for (int id = 0; id < cfgs->len; id++) {
                dynamixel_config_t cfg = cfgs->configs[id];
                dynamixel_config_t last_cfg = last_cfgs.configs[id];
                    int update_cfg = (last_cfg.kp != cfg.kp ||
                            last_cfg.ki != cfg.ki ||
                            last_cfg.kd != cfg.kd ||
                            last_cfg.compl_margin != cfg.compl_margin ||
                            last_cfg.compl_slope != cfg.compl_slope ||
                            last_cfg.LED != cfg.LED);
                if (update_cfg) {
                    arm_state->servos[id]->config (arm_state->servos[id],
                                                cfg.kp,
                                                cfg.ki,
                                                cfg.kd,
                                                cfg.compl_margin,
                                                cfg.compl_slope,
                                                cfg.LED);
                    last_cfgs.configs[id] = cfg;
                }
            }
        }


        pthread_mutex_unlock (&arm_state->serial_lock);
    }

    return NULL;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string (gopt, 'd', "device", "/dev/arm", "Device name");
    getopt_add_int (gopt, 'b', "baud", "800000", "Device baud rate");
    getopt_add_int (gopt, 'i', "initial_servo", "0", "Lowest ID servo in chain");
    getopt_add_int (gopt, 'n', "num_servos", "6", "Number of servos");
    getopt_add_string (gopt, '\0', "status-channel", "DXL_STATUS", "LCM status channel");
    getopt_add_string (gopt, '\0', "command-channel", "DXL_COMMAND", "LCM command channel");
    getopt_add_string (gopt, '\0', "config-channel", "DXL_CONFIG", "LCM config channel");



    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (-1);
    }

    printf("Openning device: %s\n", getopt_get_string (gopt, "device"));

    arm_state_t *arm_state = arm_state_create (getopt_get_string (gopt, "device"),
                                               getopt_get_int (gopt, "baud"),
                                               getopt_get_int (gopt, "initial_servo"),
                                               getopt_get_int (gopt, "num_servos"));

    // LCM Initialization
    arm_state->lcm = lcm_create (NULL);
    arm_state->command_channel = getopt_get_string (gopt, "command-channel");
    arm_state->config_channel = getopt_get_string (gopt, "config-channel");
    arm_state->status_channel = getopt_get_string (gopt, "status-channel");
    if (!arm_state->lcm)
        return -1;
    dynamixel_command_list_t_subscribe (arm_state->lcm,
                                        arm_state->command_channel,
                                        command_handler,
                                        arm_state);

    dynamixel_config_list_t_subscribe (arm_state->lcm,
                                        arm_state->config_channel,
                                        config_handler,
                                        arm_state);

    pthread_create (&arm_state->status_thread, NULL, status_loop, arm_state);
    pthread_create (&arm_state->driver_thread, NULL, driver_loop, arm_state);

    // Probably not needed, given how this operates
    pthread_join (arm_state->status_thread, NULL);
    pthread_join (arm_state->driver_thread, NULL);

    // Cleanup
    arm_state_destroy (arm_state);
    getopt_destroy (gopt);
}
