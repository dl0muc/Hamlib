/*
 *  Hamlib HAMBITS r0tor backend - main file
 *  Copyright (c) 2019 by Andreas Mueller (DC1MIL)
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <math.h>
#include <sys/time.h>
#include <time.h>

#include <hamlib/rotator.h>
#include <num_stdio.h>

#include "serial.h"
#include "misc.h"
#include "register.h"

#include "hambits.h"

struct hambits_priv_data {
  azimuth_t az;
  elevation_t el;

  struct timeval tv;	/* time last az/el update */
  azimuth_t target_az;
  elevation_t target_el;
};

/**
 * Command list:
 * 
 *
 * | Command      | Atribute | Return value     | Description                 |
 * -----------------------------------------------------------------------------
 * | setazDDD.dd; | D        | '1' == OK        | Set Target azimuth          |
 * | setelDDD.dd; | D        | '1' == OK        | Set Target elevation        |
 * | getpos;      | -        | DDD.dd;DDD.dd;   | Get position az, el         |
 * | stop;        | -        | '1' == OK        | Stop all movement and brake |
 *
 */

/**
 * hambits_transaction
 *
 * cmdstr - Command to be sent to the rig.
 * data - Buffer for reply string.  Can be NULL, indicating that no reply is
 *        is needed, but answer will still be read.
 * data_len - in: Size of buffer. It is the caller's responsibily to provide
 *            a large enough buffer for all possible replies for a command.
 *
 * returns:
 *   RIG_OK  -  if no error occurred.
 *   RIG_EIO  -  if an I/O error occurred while sending/receiving data.
 *   RIG_ETIMEOUT  -  if timeout expires without any characters received.
 */
static int hambits_transaction (ROT *rot, const char *cmdstr,
                                    char *data, size_t *data_len, size_t expected_return_length)
{
  struct rot_state *rs;
  int return_value;
  int retry_read = 0;

  rs = &rot->state;

  while(1) {
    serial_flush(&rs->rotport);

    if (cmdstr) {
      return_value = write_block(&rs->rotport, cmdstr, strlen(cmdstr));
      if (return_value != RIG_OK) {
        return return_value;
      }
    }

    /* Not all commands will send a return value, so use data = NULL if no
       return value is expected, Strings end with ';' */
    if (data != NULL) {
      memset(data,0,BUFSIZE);
      *data_len = read_string(&rs->rotport, data, expected_return_length + 1, "\n", strlen("\n"));
      if (*data_len < 0) {
        if (retry_read++ >= rot->state.rotport.retry) {
          return RIG_ETIMEOUT;
        }
      }
      else {
        return RIG_OK;
      }
    }
    else {
      return RIG_OK;
    }
  }
}

/*
 * Initialization
 */
static int hambits_init(ROT *rot)
{
  struct hambits_priv_data *priv;

  priv = (struct hambits_priv_data*)
    malloc(sizeof(struct hambits_priv_data));

  if (!priv)
    return -RIG_ENOMEM;
  rot->state.priv = (void*)priv;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);
  rot->state.rotport.type.rig = RIG_PORT_SERIAL;

  priv->az = priv->el = 0;

  priv->target_az = priv->target_el = 0;

  return RIG_OK;
}

/*
 * Cleanup
 */
static int hambits_cleanup(ROT *rot)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if (rot->state.priv)
    free(rot->state.priv);

  rot->state.priv = NULL;

  return RIG_OK;
}

/*
 * Opens the Port and sets all needed parametes for operation
 */
static int hambits_open(ROT *rot)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  /* Nothing to do here yet */
  return RIG_OK;
}

/*
 * Closes the port and stops all movement
 */
static int hambits_close(ROT *rot)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);
  /* Stop all Movement */
  return hambits_transaction(rot, "stop;" , NULL, 0, 0);
}

/*
 * Sets the target position and starts movement
 *
 * az: Target azimuth
 * el: Target elevation
 */
static int hambits_set_position(ROT *rot, azimuth_t az, elevation_t el)
{
  struct hambits_priv_data *priv = (struct hambits_priv_data *)rot->state.priv;
  char cmd_str[BUFSIZE];
  char return_str[BUFSIZE];
  size_t return_str_size;
  
  rig_debug(RIG_DEBUG_VERBOSE,"%s called: %.2f %.2f\n", __func__,
    az, el);

  priv->target_az = az;
  priv->target_el = el;

  num_sprintf(cmd_str, "setaz%03.2f;setel%03.2f;",
              az, el);

  hambits_transaction(rot, cmd_str, return_str, &return_str_size, 2);
  /* '1' == Azimuth accepted '1' == Elevation accepted  */
  rig_debug(RIG_DEBUG_VERBOSE,"Return String: %s\n", return_str);

  if(return_str_size > 0 && strstr(return_str , "11") != NULL)
    return RIG_OK;
  else
    return RIG_EINVAL;
}

/*
 * Get position of rotor
 */
static int hambits_get_position(ROT *rot, azimuth_t *az, elevation_t *el)
{
  char return_str[BUFSIZE];
  size_t return_str_size;
  char* pEnd;
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  hambits_transaction(rot, "getpos;", return_str, &return_str_size, 15);

  if(return_str_size > 8) {  /* ';' == EOS */
    *az = strtof(return_str, &pEnd);
    *el = strtof(pEnd + 1, NULL);
    rig_debug(RIG_DEBUG_VERBOSE,"Return Values: AZ: %.2f EL: %.2f\n",
    *az, *el);
    return RIG_OK;
  }
  else {
    return RIG_EINVAL;
  }
}

/*
 * Stops all movement
 */
static int hambits_stop(ROT *rot)
{
  struct hambits_priv_data *priv = (struct hambits_priv_data *)rot->state.priv;
  azimuth_t az;
  elevation_t el;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if(hambits_transaction(rot, "stop;", NULL, 0, 0) != RIG_OK)
    return RIG_EINVAL;

  if(hambits_get_position(rot, &az, &el) == RIG_OK) {
    priv->target_az = priv->az = az;
    priv->target_el = priv->el = el;
    return RIG_OK;
  }
  return RIG_EINVAL;
}

/*
 * Moves to Home Position
 */
static int hambits_park(ROT *rot)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  /* Assume home is 0,0 */
  return hambits_set_position(rot, 0, 0);
}

/*
 * Reset: Nothing to do exept parking
 */
static int hambits_reset(ROT *rot, rot_reset_t reset)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);
  
  return hambits_park(rot);
}

/*
 * Movement to direction
 */
static int hambits_move(ROT *rot, int direction, int speed)
{
  struct hambits_priv_data *priv = (struct hambits_priv_data *)rot->state.priv;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);
  rig_debug(RIG_DEBUG_TRACE, "%s: Direction = %d, Speed = %d\n", __func__, direction, speed);

  switch(direction) {
  case ROT_MOVE_UP:
    return hambits_set_position(rot, priv->target_az, 180);

  case ROT_MOVE_DOWN:
    return hambits_set_position(rot, priv->target_az, 0);

  case ROT_MOVE_CCW:
    return hambits_set_position(rot, 0, priv->target_el);

  case ROT_MOVE_CW:
    return hambits_set_position(rot, 360, priv->target_el);

  default:
    return -RIG_EINVAL;
  }

  return RIG_OK;
}

static const char *hambits_get_info(ROT *rot)
{
  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  return "Hambits r0tor: open source Arduino rotor controller.";
}

/*
 * Hambits r0tor capabilities.
 */

const struct rot_caps hambits_caps = {
  .rot_model =        ROT_MODEL_HAMBITS,
  .model_name =       "r0tor",
  .mfg_name =         "Hambits",
  .version =          "0.1",
  .copyright =        "LGPL",
  .status =           RIG_STATUS_ALPHA,
  .rot_type =         ROT_TYPE_AZEL,

  .port_type =        RIG_PORT_SERIAL,
  .serial_rate_min =  19200,
  .serial_rate_max =  19200,
  .serial_data_bits = 8,
  .serial_stop_bits = 1,
  .serial_parity =    RIG_PARITY_NONE,
  .serial_handshake = RIG_HANDSHAKE_NONE,
  .write_delay =      0,
  .post_write_delay = 0,
  .timeout =          400,
  .retry =            5,

  .min_az =      0.,
  .max_az =    360.,
  .min_el =      0.,
  .max_el =    180.,

  .priv =      NULL,  /* priv */

  .rot_init =         hambits_init,
  .rot_cleanup =      hambits_cleanup,
  .rot_open =         hambits_open,
  .rot_close =        hambits_close,

  .set_position =     hambits_set_position,
  .get_position =     hambits_get_position,
  .park =             hambits_park,
  .stop =             hambits_stop,
  .reset =            hambits_reset,
  .move =             hambits_move,

  .get_info =         hambits_get_info,
};

DECLARE_INITROT_BACKEND(hambits)
{
  rig_debug(RIG_DEBUG_VERBOSE, "hambits: _init called\n");

  rot_register(&hambits_caps);

  return RIG_OK;
}
