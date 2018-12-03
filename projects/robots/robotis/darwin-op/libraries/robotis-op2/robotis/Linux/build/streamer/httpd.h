/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#ifndef HTTPD_H_
#define HTTPD_H_

#define IO_BUFFER 256
#define BUFFER_SIZE 1024

/* the boundary is used for the M-JPEG stream, it separates the multipart stream of pictures */
#define BOUNDARY "boundarydonotcross"

/*
 * this defines the buffer size for a JPG-frame
 * selecting to large values will allocate much wasted RAM for each buffer
 * selecting to small values will lead to crashes due to to small buffers
 */
#define MAX_FRAME_SIZE (256*1024)
#define TEN_K (10*1024)


#define ABS(a) (((a) < 0) ? -(a) : (a))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define LENGTH_OF(x) (sizeof(x)/sizeof(x[0]))

#ifdef DEBUG
#define DBG(...) fprintf(stderr, " DBG(%s, %s(), %d): ", __FILE__, __FUNCTION__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...)
#endif

#define LOG(...) { char _bf[1024] = {0}; snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); fprintf(stderr, "%s", _bf); syslog(LOG_INFO, "%s", _bf); }

#define OUTPUT_PLUGIN_PREFIX " o: "
#define OPRINT(...) { char _bf[1024] = {0}; snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); fprintf(stderr, "%s", OUTPUT_PLUGIN_PREFIX); fprintf(stderr, "%s", _bf); syslog(LOG_INFO, "%s", _bf); }


/*
 * Standard header to be send along with other header information like mimetype.
 *
 * The parameters should ensure the browser does not cache our answer.
 * A browser should connect for each file and not serve files from his cache.
 * Using cached pictures would lead to showing old/outdated pictures
 * Many browser seem to ignore, or at least not always obey those headers
 * since i observed caching of files from time to time.
 */
#define STD_HEADER "Connection: close\r\n" \
                   "Server: MJPG-Streamer/0.2\r\n" \
                   "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" \
                   "Pragma: no-cache\r\n" \
                   "Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"


typedef enum{
    IN_CMD_UNKNOWN = 0,
    IN_CMD_HELLO,
    IN_CMD_RELOAD,
    IN_CMD_SAVE,
    IN_CMD_GAIN_PLUS,
    IN_CMD_GAIN_MINUS,
    IN_CMD_EXPOSURE_PLUS,
    IN_CMD_EXPOSURE_MINUS,
    IN_CMD_HUE_SET,
    IN_CMD_HUE_PLUS,
    IN_CMD_HUE_MINUS,
    IN_CMD_TOLERANCE_SET,
    IN_CMD_TOLERANCE_PLUS,
    IN_CMD_TOLERANCE_MINUS,
    IN_CMD_MIN_SATURATION_SET,
    IN_CMD_MIN_SATURATION_PLUS,
    IN_CMD_MIN_SATURATION_MINUS,
    IN_CMD_MIN_VALUE_SET,
    IN_CMD_MIN_VALUE_PLUS,
    IN_CMD_MIN_VALUE_MINUS,
    
    /* walk tuner */
    IN_CMD_WALK_MODE,
    IN_CMD_WALK_SAVE,
    IN_CMD_WALK_X_OFFSET,
    IN_CMD_WALK_Y_OFFSET,
    IN_CMD_WALK_Z_OFFSET,
    IN_CMD_WALK_ROLL_OFFSET,
    IN_CMD_WALK_PITCH_OFFSET,
    IN_CMD_WALK_YAW_OFFSET,
    IN_CMD_WALK_HIP_OFFSET,
    IN_CMD_WALK_AUTO_BALANCE,
    IN_CMD_WALK_PERIOD_TIME,
    IN_CMD_WALK_DSP_RATIO,
    IN_CMD_WALK_STEP_FB_RATIO,
    IN_CMD_WALK_STEP_FB,
    IN_CMD_WALK_STEP_RL,
    IN_CMD_WALK_STEP_DIR,
    IN_CMD_WALK_TURN_AIM,
    IN_CMD_WALK_FOOT_HEIGHT,
    IN_CMD_WALK_SWING_RL,
    IN_CMD_WALK_SWING_TD,
    IN_CMD_WALK_PELVIS_OFFSET,
    IN_CMD_WALK_ARM_SWING_GAIN,
    IN_CMD_WALK_B_KNEE_GAIN,
    IN_CMD_WALK_B_ANKLE_PITCH_GAIN,
    IN_CMD_WALK_B_HIP_ROLL_GAIN,
    IN_CMD_WALK_B_ANKLE_ROLL_GAIN,
    IN_CMD_WALK_P_GAIN,
    IN_CMD_WALK_I_GAIN,
    IN_CMD_WALK_D_GAIN,
    
}in_cmd_type;

/* commands which can be send to the input plugin */
typedef enum {
  OUT_CMD_UNKNOWN = 0,
  OUT_CMD_HELLO
}out_cmd_type;


typedef struct _globals globals;
struct _globals {
    /* signal fresh frames */
    pthread_mutex_t db;
    pthread_cond_t  db_update;

    /* global JPG frame, this is more or less the "database" */
    unsigned char* buf;
    int size;
};




/*
 * Only the following fileypes are supported.
 *
 * Other filetypes are simply ignored!
 * This table is a 1:1 mapping of files extension to a certain mimetype.
 */
static const struct {
  const char *dot_extension;
  const char *mimetype;
} mimetypes[] = {
  { ".html", "text/html" },
  { ".htm",  "text/html" },
  { ".css",  "text/css" },
  { ".js",   "text/javascript" },
  { ".txt",  "text/plain" },
  { ".jpg",  "image/jpeg" },
  { ".jpeg", "image/jpeg" },
  { ".png",  "image/png"},
  { ".gif",  "image/gif" },
  { ".ico",  "image/x-icon" },
  { ".swf",  "application/x-shockwave-flash" },
  { ".cab",  "application/x-shockwave-flash" },
  { ".jar",  "application/java-archive" }
};

/*
 * mapping between command string and command type
 * it is used to find the command for a certain string
 */
static const struct {
  const char *string;
  const in_cmd_type cmd;
} in_cmd_mapping[] = {
  { "reload", IN_CMD_RELOAD },
  { "save", IN_CMD_SAVE },
  { "gain_plus", IN_CMD_GAIN_PLUS },
  { "gain_minus", IN_CMD_GAIN_MINUS },
  { "exposure_plus", IN_CMD_EXPOSURE_PLUS },
  { "exposure_minus", IN_CMD_EXPOSURE_MINUS },
  { "hue_set", IN_CMD_HUE_SET },
  { "hue_plus", IN_CMD_HUE_PLUS },
  { "hue_minus", IN_CMD_HUE_MINUS },
  { "tolerance_set", IN_CMD_TOLERANCE_SET },
  { "tolerance_plus", IN_CMD_TOLERANCE_PLUS },
  { "tolerance_minus", IN_CMD_TOLERANCE_MINUS },
  { "min_saturation_set", IN_CMD_MIN_SATURATION_SET },
  { "min_saturation_plus", IN_CMD_MIN_SATURATION_PLUS },
  { "min_saturation_minus", IN_CMD_MIN_SATURATION_MINUS },
  { "min_value_set", IN_CMD_MIN_VALUE_SET },
  { "min_value_plus", IN_CMD_MIN_VALUE_PLUS },
  { "min_value_minus", IN_CMD_MIN_VALUE_MINUS },
  
  { "walk_mode", IN_CMD_WALK_MODE },
  { "walk_save", IN_CMD_WALK_SAVE },
  { "walk_x_offset", IN_CMD_WALK_X_OFFSET },
  { "walk_y_offset", IN_CMD_WALK_Y_OFFSET },
  { "walk_z_offset", IN_CMD_WALK_Z_OFFSET },
  { "walk_roll_offset", IN_CMD_WALK_ROLL_OFFSET },
  { "walk_pitch_offset", IN_CMD_WALK_PITCH_OFFSET },
  { "walk_yaw_offset", IN_CMD_WALK_YAW_OFFSET },
  { "walk_hip_offset", IN_CMD_WALK_HIP_OFFSET },
  { "walk_auto_balance", IN_CMD_WALK_AUTO_BALANCE },
  { "walk_period_time", IN_CMD_WALK_PERIOD_TIME },
  { "walk_dsp_ratio", IN_CMD_WALK_DSP_RATIO },
  { "walk_step_fb_ratio", IN_CMD_WALK_STEP_FB_RATIO },
  { "walk_step_fb", IN_CMD_WALK_STEP_FB },
  { "walk_step_rl", IN_CMD_WALK_STEP_RL },
  { "walk_step_dir", IN_CMD_WALK_STEP_DIR },
  { "walk_turn_aim", IN_CMD_WALK_TURN_AIM },
  { "walk_foot_height", IN_CMD_WALK_FOOT_HEIGHT },
  { "walk_swing_rl", IN_CMD_WALK_SWING_RL },
  { "walk_swing_td", IN_CMD_WALK_SWING_TD },
  { "walk_pelvis_offset", IN_CMD_WALK_PELVIS_OFFSET },
  { "walk_arm_swing_gain", IN_CMD_WALK_ARM_SWING_GAIN },
  { "walk_b_knee_gain", IN_CMD_WALK_B_KNEE_GAIN },
  { "walk_b_ankle_pitch_gain", IN_CMD_WALK_B_ANKLE_PITCH_GAIN },
  { "walk_b_hip_roll_gain", IN_CMD_WALK_B_HIP_ROLL_GAIN },
  { "walk_b_ankle_roll_gain", IN_CMD_WALK_B_ANKLE_ROLL_GAIN },
  { "walk_p_gain", IN_CMD_WALK_P_GAIN },
  { "walk_i_gain", IN_CMD_WALK_I_GAIN },
  { "walk_d_gain", IN_CMD_WALK_D_GAIN },
};


/* mapping between command string and command type */
static const struct {
  const char *string;
  const out_cmd_type cmd;
} out_cmd_mapping[] = {
  { "hello_output", OUT_CMD_HELLO }
};

/* the webserver determines between these values for an answer */
typedef enum { A_UNKNOWN, A_SNAPSHOT, A_STREAM, A_COMMAND, A_FILE } answer_t;

/*
 * the client sends information with each request
 * this structure is used to store the important parts
 */
typedef struct {
  answer_t type;
  char *parameter;
  char *client;
  char *credentials;
} request;

/* the iobuffer structure is used to read from the HTTP-client */
typedef struct {
  int level;              /* how full is the buffer */
  char buffer[IO_BUFFER]; /* the data */
} iobuffer;

/* store configuration for each server instance */
typedef struct {
  int port;
  char *credentials;
  char *www_folder;
  char nocommands;
} config;

/* context of each server thread */
typedef struct {
  int sd;
  globals *pglobal;
  pthread_t threadID;

  config conf;
} context;

/*
 * this struct is just defined to allow passing all necessary details to a worker thread
 * "cfd" is for connected/accepted filedescriptor
 */
typedef struct {
  context *pc;
  int fd;
} cfd;

#include "ColorFinder.h"

using namespace Robot;

class httpd
{
private:
    static globals* pglobal;
    static context* server;

    static void init_iobuffer(iobuffer *iobuf);
    static void init_request(request *req);
    static void free_request(request *req);
    static int _read(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout);
    static int _readline(int fd, iobuffer *iobuf, void *buffer, size_t len, int timeout);
    static void decodeBase64(char *data);
    static void send_snapshot(int fd);
    static void send_stream(int fd);
    static void send_file(int fd, char *parameter);
    static void command(int fd, char *parameter);
    static void input_cmd(in_cmd_type cmd, float value, char* res_str);
    static void server_cleanup(void *arg);
    static void *client_thread( void *arg );

public:
    static ColorFinder* finder;
    static char*        ini_section;
    static ColorFinder* ball_finder;
    static ColorFinder* red_finder;
    static ColorFinder* yellow_finder;
    static ColorFinder* blue_finder;
    static minIni*      ini;
	static bool ClientRequest;

    static void *server_thread( void *arg );
    static void send_error(int fd, int which, char *message);
};


#endif /* HTTPD_H_ */




