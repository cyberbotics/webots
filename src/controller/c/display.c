/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/display.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "display_private.h"
#include "g_image.h"
#include "messages.h"
#include "remote_control_private.h"  //required to forward the display state from Webots to the remote_control
#include "robot_private.h"

typedef struct WbImageStructPrivate {
  int id;
  WbDeviceTag device_tag;
} WbImageStruct;

// WbImageRef handle message
typedef struct {
  int id;
  int x;
  int y;
  int width;
  int height;
  int format;
  bool blend;
  void *image;
} DisplayImageMessage;

// a drawing primitive message
typedef struct {
  bool pfill;
  int *px;
  int *py;
  int psize;
  char *ptxt;
} DisplayPrimitiveMessage;

typedef struct {
  char *name;
  int size;
  bool anti_aliasing;
} DisplayFontProperties;

// a drawing property message
typedef struct {
  union {
    int color;
    double alpha;
    double opacity;
    DisplayFontProperties font_properties;
  };
} DisplayPropertyMessage;

typedef struct _SaveOrder_ {
  int id;
  char *filename;
  unsigned char type;
  struct _SaveOrder_ *next;
} SaveOrder;

// a message contains either a drawing primitive
// or either a drawing property
typedef struct _DisplayMessage_ {
  int message;
  union {
    DisplayPrimitiveMessage *primitive_message;
    DisplayPropertyMessage *property_message;
    DisplayImageMessage *image_message;
  };
  struct _DisplayMessage_ *next;
} DisplayMessage;

typedef struct {
  int width;
  int height;
  int image_next_free_id;
  DisplayMessage *messages_head;  // list of messages to send
  DisplayMessage *messages_tail;
  SaveOrder *save_orders;  // list of images which will be saved
  bool hasBeenUsed;
  WbDeviceTag camera_to_attach;
  bool is_camera_attached;
  bool camera_to_detach;
} Display;

static Display *wb_display_create(int width, int height) {
  Display *display = malloc(sizeof(Display));
  display->width = width;
  display->height = height;
  display->image_next_free_id = 1;  // 0 is reserved for current display content
  display->messages_head = NULL;
  display->messages_tail = NULL;
  display->save_orders = NULL;
  display->hasBeenUsed = false;
  display->camera_to_attach = 0;
  display->is_camera_attached = false;
  display->camera_to_detach = false;
  return display;
}

static inline void message_enqueue(Display *d, DisplayMessage *new) {
  new->next = NULL;
  if (d->messages_head == NULL)
    d->messages_head = new;
  else
    d->messages_tail->next = new;
  d->messages_tail = new;
}

static Display *wb_display_get_struct(WbDeviceTag tag) {
  WbDevice *d = robot_get_device_with_node(tag, WB_NODE_DISPLAY, true);
  return d ? d->pdata : NULL;
}

int display_get_channel_number(int pixel_format) {
  switch (pixel_format) {
    case WB_IMAGE_RGB:
      return 3;
    case WB_IMAGE_RGBA:
    case WB_IMAGE_ARGB:
    case WB_IMAGE_BGRA:
    case WB_IMAGE_ABGR:
      return 4;
    default:
      assert(0);
  }
  return 0;
}

static bool save_image(Display *d, int id, int width, int height, unsigned char *im) {
  SaveOrder *order = d->save_orders;
  if (order && order->id == id) {
    GImage *i = (GImage *)malloc(sizeof(GImage));
    i->width = width;
    i->height = height;
    i->failed = 0;
    i->flipped = 0;
    i->data_format = G_IMAGE_DATA_FORMAT_BGRA;
    i->data = im;
    g_image_save(i, order->filename, 100);
    free(i);
    free(order->filename);
    d->save_orders = d->save_orders->next;
    free(order);
    return true;
  } else
    return false;
}

static void wb_display_read_answer(WbDevice *d, WbRequest *r) {
  int width, height, id;
  unsigned char *im;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:
      width = request_read_uint16(r);
      height = request_read_uint16(r);
      d->pdata = wb_display_create(width, height);
      break;
    case C_DISPLAY_IMAGE_SAVE:
      id = request_read_int32(r);
      width = request_read_int32(r);
      height = request_read_int32(r);
      im = request_read_data(r, 4 * width * height);
      save_image(d->pdata, id, width, height, im);
      break;
    case C_DISPLAY_IMAGE_GET_ALL:
      r->pointer--;  // unread command
      remote_control_handle_one_message(r, robot_get_device_tag(d));
      break;
    default:
      ROBOT_ASSERT(0);
      break;
  }
}

static void wb_display_write_request(WbDevice *d, WbRequest *r) {
  Display *display = d->pdata;
  DisplayMessage *current_message = display->messages_head;

  if (display->camera_to_attach != 0) {
    request_write_uchar(r, C_DISPLAY_ATTACH_CAMERA);
    request_write_uint16(r, display->camera_to_attach);
    display->camera_to_attach = 0;
  } else if (display->camera_to_detach) {
    request_write_uchar(r, C_DISPLAY_DETACH_CAMERA);
    display->camera_to_detach = false;
  }

  // for each element of the request list, write a request
  while (current_message) {
    display->hasBeenUsed = true;
    int message = current_message->message;
    request_write_uchar(r, message);
    if (message == C_DISPLAY_SET_COLOR) {
      request_write_int32(r, current_message->property_message->color);
      free(current_message->property_message);
    } else if (message == C_DISPLAY_SET_ALPHA) {
      request_write_uchar(r, (unsigned char)(current_message->property_message->alpha * 255));
      free(current_message->property_message);
    } else if (message == C_DISPLAY_SET_OPACITY) {
      request_write_uchar(r, (unsigned char)(current_message->property_message->opacity * 255));
      free(current_message->property_message);
    } else if (message == C_DISPLAY_SET_FONT) {
      request_write_uint32(r, current_message->property_message->font_properties.size);
      request_write_uchar(r, current_message->property_message->font_properties.anti_aliasing ? 1 : 0);
      request_write_uint16(r, strlen(current_message->property_message->font_properties.name) + 1);
      request_write_string(r, current_message->property_message->font_properties.name);
      free(current_message->property_message->font_properties.name);
      free(current_message->property_message);
    } else if (message & 0x20) {  // a drawing primitive
      request_write_int16(r, current_message->primitive_message->psize);
      request_write_data(r, current_message->primitive_message->px, current_message->primitive_message->psize * sizeof(int));
      request_write_data(r, current_message->primitive_message->py, current_message->primitive_message->psize * sizeof(int));
      if (message == C_DISPLAY_DRAW_TEXT)
        request_write_string(r, current_message->primitive_message->ptxt);
      else if (message == C_DISPLAY_DRAW_RECTANGLE || message == C_DISPLAY_DRAW_OVAL || message == C_DISPLAY_DRAW_POLYGON)
        request_write_uchar(r, current_message->primitive_message->pfill ? 1 : 0);
      free(current_message->primitive_message->px);
      free(current_message->primitive_message->py);
      free(current_message->primitive_message->ptxt);
      free(current_message->primitive_message);
    } else if (message & 0x40) {  // WbImageRef handle message
      request_write_int32(r, current_message->image_message->id);
      if (message == C_DISPLAY_IMAGE_PASTE || message == C_DISPLAY_IMAGE_COPY) {
        request_write_int16(r, current_message->image_message->x);
        request_write_int16(r, current_message->image_message->y);
      }
      if (message == C_DISPLAY_IMAGE_PASTE)
        request_write_uchar(r, current_message->image_message->blend ? 1 : 0);
      if (message == C_DISPLAY_IMAGE_COPY || message == C_DISPLAY_IMAGE_LOAD) {
        request_write_int16(r, current_message->image_message->width);
        request_write_int16(r, current_message->image_message->height);
      }
      if (message == C_DISPLAY_IMAGE_LOAD) {
        request_write_uchar(r, current_message->image_message->format);
        request_write_data(r, current_message->image_message->image,
                           current_message->image_message->width * current_message->image_message->height *
                             display_get_channel_number(current_message->image_message->format));
        free(current_message->image_message->image);
      }
      free(current_message->image_message);
    }
    DisplayMessage *tmp = current_message->next;
    free(current_message);
    current_message = tmp;
  }
  display->messages_head = NULL;
  display->messages_tail = NULL;
}

static void wb_display_cleanup(WbDevice *d) {
  Display *dp = d->pdata;
  DisplayMessage *current_message;
  while (dp->messages_head) {
    current_message = dp->messages_head->next;
    if (dp->messages_head->message & 0x20) {
      free(dp->messages_head->primitive_message->px);
      free(dp->messages_head->primitive_message->py);
      free(dp->messages_head->primitive_message->ptxt);
      free(dp->messages_head->primitive_message);
    } else if (dp->messages_head->message & 0x40) {
      if (dp->messages_head->message == C_DISPLAY_IMAGE_LOAD)
        free(dp->messages_head->image_message->image);
      free(dp->messages_head->image_message);
    } else {
      free(dp->messages_head->property_message);
    }
    free(dp->messages_head);
    dp->messages_head = current_message;
  }
}

// add a drawing primitive request into Display's messages
static void wb_display_draw_primitive(WbDeviceTag tag, int primitive, const int *x, const int *y, int size, bool fill,
                                      const char *txt) {
  assert(x && y && size > 0);
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayPrimitiveMessage *p = (DisplayPrimitiveMessage *)malloc(sizeof(DisplayPrimitiveMessage));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (d && m && p) {
    m->message = primitive;
    p->pfill = fill;
    p->psize = size;
    p->px = (int *)malloc(size * sizeof(int));
    p->py = (int *)malloc(size * sizeof(int));
    memcpy(p->px, x, size * sizeof(int));
    memcpy(p->py, y, size * sizeof(int));
    if (txt) {
      p->ptxt = (char *)malloc(strlen(txt) + 1);
      if (p->ptxt)
        strcpy(p->ptxt, txt);
    } else
      p->ptxt = NULL;
    m->primitive_message = p;
    message_enqueue(d, m);
  } else {
    free(m);
    free(p);
  }
  robot_mutex_unlock();
}

// add a drawing property request into Display's messages
static void wb_display_set_property(WbDeviceTag tag, int primitive, void *data, void *font_size, void *anti_aliasing) {
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayPropertyMessage *p = (DisplayPropertyMessage *)malloc(sizeof(DisplayPropertyMessage));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (d && m && p) {
    m->message = primitive;
    switch (primitive) {
      case C_DISPLAY_SET_COLOR:
        p->color = *((int *)data);
        break;
      case C_DISPLAY_SET_ALPHA:
        p->alpha = *((double *)data);
        break;
      case C_DISPLAY_SET_OPACITY:
        p->opacity = *((double *)data);
        break;
      case C_DISPLAY_SET_FONT: {
        const int size = strlen((char *)data) + 1;
        p->font_properties.name = malloc(size);
        memcpy(p->font_properties.name, (char *)data, size);
        p->font_properties.size = *((int *)font_size);
        p->font_properties.anti_aliasing = *((bool *)anti_aliasing);
        break;
      }
    }
    m->property_message = p;
    message_enqueue(d, m);
  } else {
    free(m);
    free(p);
  }
  robot_mutex_unlock();
}

static void display_toggle_remote(WbDevice *d, WbRequest *r) {
  const Display *display = d->pdata;
  if (display->hasBeenUsed)
    request_write_uchar(r, C_DISPLAY_IMAGE_GET_ALL);
}

void wbr_display_save_image(WbDeviceTag tag, int id, int width, int height, unsigned char *image) {
  Display *d = wb_display_get_struct(tag);
  if (d) {
    if (!save_image(d, id, width, height, image))
      fprintf(stderr, "%s(): wrong id.\n", __FUNCTION__);
  } else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
}

// Protected functions (exported to device.cc)

void wb_display_init(WbDevice *d) {
  d->pdata = NULL;
  d->write_request = wb_display_write_request;
  d->read_answer = wb_display_read_answer;
  d->cleanup = wb_display_cleanup;
  d->toggle_remote = display_toggle_remote;
}

// Public function available from the user API

int wb_display_get_height(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  const Display *d = wb_display_get_struct(tag);
  if (d)
    result = d->height;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

int wb_display_get_width(WbDeviceTag tag) {
  int result = -1;
  robot_mutex_lock();
  const Display *d = wb_display_get_struct(tag);
  if (d)
    result = d->width;
  else
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  robot_mutex_unlock();
  return result;
}

void wb_display_set_color(WbDeviceTag tag, int color) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (color > 0xFFFFFF || color < 0) {
    fprintf(stderr, "Error: %s(): 'color' argument out of bounds.\n", __FUNCTION__);
    return;
  }
  wb_display_set_property(tag, C_DISPLAY_SET_COLOR, &color, NULL, NULL);
}

void wb_display_set_alpha(WbDeviceTag tag, double alpha) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (alpha > 1.0 || alpha < 0.0) {
    fprintf(stderr, "Error: %s(): 'alpha' argument out of bounds.\n", __FUNCTION__);
    return;
  }
  wb_display_set_property(tag, C_DISPLAY_SET_ALPHA, &alpha, NULL, NULL);
}

void wb_display_set_opacity(WbDeviceTag tag, double opacity) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (opacity > 1.0 || opacity < 0.0) {
    fprintf(stderr, "Error: %s(): 'opacity' argument out of bounds.\n", __FUNCTION__);
    return;
  }
  wb_display_set_property(tag, C_DISPLAY_SET_OPACITY, &opacity, NULL, NULL);
}

void wb_display_set_font(WbDeviceTag tag, const char *font, int size, bool anti_aliasing) {
  if (size <= 0) {
    fprintf(stderr, "Error: %s(): 'size' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  robot_mutex_lock();
  const Display *display = wb_display_get_struct(tag);
  if (!display) {
    fprintf(stderr, "Error: %s(): invalid display.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  robot_mutex_unlock();
  wb_display_set_property(tag, C_DISPLAY_SET_FONT, (char *)font, &size, &anti_aliasing);
}

void wb_display_attach_camera(WbDeviceTag tag, WbDeviceTag camera_tag) {
  robot_mutex_lock();
  Display *display = wb_display_get_struct(tag);
  const WbDevice *camera = robot_get_device_with_node(camera_tag, WB_NODE_CAMERA, true);
  if (!display) {
    fprintf(stderr, "Error: %s(): invalid display.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  if (!camera) {
    fprintf(stderr, "Error: %s(): invalid camera.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  if (display->is_camera_attached) {
    fprintf(stderr, "Error: %s(): a camera is already attached to the display.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  display->camera_to_attach = camera_tag;
  display->is_camera_attached = true;
  display->camera_to_detach = false;
  robot_mutex_unlock();
}

void wb_display_detach_camera(WbDeviceTag tag) {
  robot_mutex_lock();
  Display *display = wb_display_get_struct(tag);
  if (display == NULL) {
    fprintf(stderr, "Error: %s(): invalid display.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  if (!display->is_camera_attached) {
    fprintf(stderr, "Error: %s(): no camera to detach.\n", __FUNCTION__);
    robot_mutex_unlock();
    return;
  }
  display->camera_to_attach = 0;
  display->is_camera_attached = false;
  display->camera_to_detach = true;
  robot_mutex_unlock();
}

void wb_display_draw_pixel(WbDeviceTag tag, int x, int y) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }

  const int px[] = {x};
  const int py[] = {y};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_PIXEL, px, py, 1, false, NULL);
}

void wb_display_draw_line(WbDeviceTag tag, int x1, int y1, int x2, int y2) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  const int px[] = {x1, x2};
  const int py[] = {y1, y2};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_LINE, px, py, 2, false, NULL);
}

void wb_display_draw_rectangle(WbDeviceTag tag, int x, int y, int width, int height) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (width <= 0) {
    fprintf(stderr, "Error: %s(): 'width' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  if (height <= 0) {
    fprintf(stderr, "Error: %s(): 'height' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  const int px[] = {x, width};
  const int py[] = {y, height};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_RECTANGLE, px, py, 2, false, NULL);
}

void wb_display_draw_oval(WbDeviceTag tag, int cx, int cy, int a, int b) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (a <= 0) {
    fprintf(stderr, "Error: %s(): 'horizontal_radius' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  if (b <= 0) {
    fprintf(stderr, "Error: %s(): 'vertical_radius' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  const int px[] = {cx, a};
  const int py[] = {cy, b};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_OVAL, px, py, 2, false, NULL);
}

void wb_display_draw_polygon(WbDeviceTag tag, const int *x, const int *y, int size) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (!x || !y) {
    fprintf(stderr, "Error: %s(): 'x' or 'y' argument is NULL.\n", __FUNCTION__);
    return;
  }
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_POLYGON, x, y, size, false, NULL);
}

void wb_display_draw_text(WbDeviceTag tag, const char *text, int x, int y) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (!text || strlen(text) == 0) {
    fprintf(stderr, "Error: %s(): 'text' argument is NULL or empty.\n", __FUNCTION__);
    return;
  }
  const int px[] = {x};
  const int py[] = {y};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_TEXT, px, py, 1, false, text);
}

void wb_display_fill_rectangle(WbDeviceTag tag, int x, int y, int width, int height) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (width <= 0) {
    fprintf(stderr, "Error: %s(): 'width' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  if (height <= 0) {
    fprintf(stderr, "Error: %s(): 'height' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  const int px[] = {x, width};
  const int py[] = {y, height};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_RECTANGLE, px, py, 2, true, NULL);
}

void wb_display_fill_oval(WbDeviceTag tag, int cx, int cy, int a, int b) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (a <= 0) {
    fprintf(stderr, "Error: %s(): 'horizontal_radius' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  if (b <= 0) {
    fprintf(stderr, "Error: %s(): 'vertical_radius' argument is negative or null.\n", __FUNCTION__);
    return;
  }
  const int px[] = {cx, a};
  const int py[] = {cy, b};
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_OVAL, px, py, 2, true, NULL);
}

void wb_display_fill_polygon(WbDeviceTag tag, const int *x, const int *y, int size) {
  const Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return;
  }
  if (!x || !y) {
    fprintf(stderr, "Error: %s(): 'x' or 'y' arguments is NULL.\n", __FUNCTION__);
    return;
  }
  wb_display_draw_primitive(tag, C_DISPLAY_DRAW_POLYGON, x, y, size, true, NULL);
}

WbImageRef wb_display_image_copy(WbDeviceTag tag, int x, int y, int width, int height) {
  if (width < 1 || height < 1) {
    fprintf(stderr, "Error: %s(): 'width' or 'height' argument is invalid.\n", __FUNCTION__);
    return NULL;
  }
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  WbImageStruct *im = (WbImageStruct *)malloc(sizeof(WbImageStruct));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    free(m);
    free(i);
    free(im);
    im = NULL;
  } else if (m && i && im) {
    m->message = C_DISPLAY_IMAGE_COPY;
    i->id = d->image_next_free_id;
    i->x = x;
    i->y = y;
    i->width = width;
    i->height = height;
    m->image_message = i;
    message_enqueue(d, m);
    im->id = d->image_next_free_id;
    im->device_tag = tag;
    d->image_next_free_id++;
  }
  robot_mutex_unlock();
  return im;
}

void wb_display_image_paste(WbDeviceTag tag, WbImageRef ir, int x, int y, bool blend) {
  if (!ir || ir->id <= 0) {
    fprintf(stderr, "Error: %s(): invalid WbImageRef argument.\n", __FUNCTION__);
    return;
  }
  if (ir->device_tag != tag) {
    fprintf(stderr, "Error: %s(): invalid WbImageRef created by a different Display device.\n", __FUNCTION__);
    return;
  }
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    free(m);
    free(i);
  } else if (m && i) {
    m->message = C_DISPLAY_IMAGE_PASTE;
    i->id = ir->id;
    i->x = x;
    i->y = y;
    i->blend = blend;
    m->image_message = i;
    message_enqueue(d, m);
  }
  robot_mutex_unlock();
}

WbImageRef wb_display_image_new(WbDeviceTag tag, int width, int height, const void *data, int format) {
  if (!data) {
    fprintf(stderr, "Error: %s(): 'data' argument is NULL.\n", __FUNCTION__);
    return NULL;
  }
  if (width < 1 || height < 1) {
    fprintf(stderr, "Error: %s(): 'width' or 'height' argument is invalid.\n", __FUNCTION__);
    return NULL;
  }
  if (format != WB_IMAGE_RGB && format != WB_IMAGE_RGBA && format != WB_IMAGE_ARGB && format != WB_IMAGE_BGRA &&
      format != WB_IMAGE_ABGR) {
    fprintf(stderr, "Error: %s(): 'format' argument is invalid.\n", __FUNCTION__);
    return NULL;
  }
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    return NULL;
  }

  robot_mutex_lock();
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  WbImageStruct *im = (WbImageStruct *)malloc(sizeof(WbImageStruct));
  m->message = C_DISPLAY_IMAGE_LOAD;
  m->image_message = i;
  message_enqueue(d, m);
  i->id = d->image_next_free_id;
  i->width = width;
  i->height = height;
  i->format = format;
  i->image = malloc(i->width * i->height * display_get_channel_number(i->format));

  if (display_get_channel_number(i->format) == 3)
    memcpy(i->image, data, i->width * i->height * display_get_channel_number(i->format));
  else {  // channel == 4
    int j;
    const int s = width * height;
    const unsigned char *img = (unsigned char *)data;
    for (j = 0; j < s; j++)
      ((uint32_t *)i->image)[j] = img[j * 4 + 3] << 24 | img[j * 4 + 2] << 16 | img[j * 4 + 1] << 8 | img[j * 4];
  }

  im->id = d->image_next_free_id;
  im->device_tag = tag;
  d->image_next_free_id++;
  robot_mutex_unlock();
  return im;
}

WbImageRef wb_display_image_load(WbDeviceTag tag, const char *filename) {
  if (!filename || strlen(filename) == 0) {
    fprintf(stderr, "Error: %s(): 'filename' argument is NULL or empty.\n", __FUNCTION__);
    return NULL;
  }
  GImage *gi = g_image_new(filename);
  if (gi->failed || (gi->data_format != G_IMAGE_DATA_FORMAT_ABGR && gi->data_format != G_IMAGE_DATA_FORMAT_RGBA &&
                     gi->data_format != G_IMAGE_DATA_FORMAT_RGB)) {
    fprintf(stderr, "Error: %s(): the \"%s\" image is unreadable.\n", __FUNCTION__, filename);
    g_image_delete(gi);
    return NULL;
  }
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  WbImageStruct *im = (WbImageStruct *)malloc(sizeof(WbImageStruct));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    g_image_delete(gi);
    free(m);
    free(i);
    free(im);
    im = NULL;
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  } else if (m && i && im) {
    m->message = C_DISPLAY_IMAGE_LOAD;
    i->id = d->image_next_free_id;
    switch (gi->data_format) {
      case G_IMAGE_DATA_FORMAT_RGB:
        i->format = WB_IMAGE_RGB;
        break;
      case G_IMAGE_DATA_FORMAT_RGBA:
        i->format = WB_IMAGE_RGBA;
        break;
      case G_IMAGE_DATA_FORMAT_ABGR:
        i->format = WB_IMAGE_ABGR;
        break;
      default:
        assert(false);
    }
    i->width = gi->width;
    i->height = gi->height;
    i->image = gi->data;
    m->image_message = i;
    message_enqueue(d, m);
    im->id = d->image_next_free_id;
    im->device_tag = tag;
    d->image_next_free_id++;
    free(gi);  // this should not delete the image data
  }
  robot_mutex_unlock();
  return im;
}

void wb_display_image_save(WbDeviceTag tag, WbImageRef ir, const char *filename) {
  if (!filename || strlen(filename) == 0) {
    fprintf(stderr, "Error: %s(): 'filename' argument is NULL or empty.\n", __FUNCTION__);
    return;
  }
  if (ir) {
    if (ir->id <= 0) {
      fprintf(stderr, "Error: %s(): invalid WbImageRef.\n", __FUNCTION__);
      return;
    }
    if (ir->device_tag != tag) {
      fprintf(stderr, "Error: %s(): invalid WbImageRef created by a different Display device.\n", __FUNCTION__);
      return;
    }
  }
  unsigned char type = g_image_get_type(filename);
  if (type != G_IMAGE_PNG && type != G_IMAGE_JPEG) {
    fprintf(stderr, "Error: %s(): unsupported file format. Supported file formats are \".jpg\" and \".png\".\n", __FUNCTION__);
    return;
  }
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  SaveOrder *o = (SaveOrder *)malloc(sizeof(SaveOrder));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    free(m);
    free(o);
    free(i);
  } else if (m && i && o) {
    const int id = ir ? ir->id : 0;
    i->id = id;
    o->id = id;
    o->filename = (char *)malloc(strlen(filename) + 1);
    o->type = type;
    strcpy(o->filename, filename);
    o->next = d->save_orders;
    m->message = C_DISPLAY_IMAGE_SAVE;
    m->image_message = i;
    message_enqueue(d, m);
    d->save_orders = o;
  }
  wb_robot_flush_unlocked(__FUNCTION__);
  robot_mutex_unlock();
}

void wb_display_image_delete(WbDeviceTag tag, WbImageRef ir) {
  if (!ir || ir->id <= 0) {
    fprintf(stderr, "Error: %s(): invalid WbImageRef.\n", __FUNCTION__);
    return;
  }
  if (ir->device_tag != tag) {
    fprintf(stderr, "Error: %s(): invalid WbImageRef created by a different Display device.\n", __FUNCTION__);
    return;
  }
  DisplayMessage *m = (DisplayMessage *)malloc(sizeof(DisplayMessage));
  DisplayImageMessage *i = (DisplayImageMessage *)malloc(sizeof(DisplayImageMessage));
  robot_mutex_lock();
  Display *d = wb_display_get_struct(tag);
  if (!d) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    free(m);
    free(i);
  } else if (m && i) {
    m->message = C_DISPLAY_IMAGE_DELETE;
    i->id = ir->id;
    m->image_message = i;
    message_enqueue(d, m);
  }
  free(ir);
  robot_mutex_unlock();
}
