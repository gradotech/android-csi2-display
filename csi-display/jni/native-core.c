#include <string.h>
#include <stdint.h>
#include <jni.h>
#include <android/log.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <gst/app/gstappsrc.h>
#include <linux/gpio.h>

GST_DEBUG_CATEGORY_STATIC (debug_category);
#define GST_CAT_DEFAULT debug_category

#if GLIB_SIZEOF_VOID_P == 8
# define GET_CUSTOM_DATA(env, thiz, fieldID) (CustomData *)(*env)->GetLongField (env, thiz, fieldID)
# define SET_CUSTOM_DATA(env, thiz, fieldID, data) (*env)->SetLongField (env, thiz, fieldID, (jlong)data)
#else
# define GET_CUSTOM_DATA(env, thiz, fieldID) (CustomData *)(jint)(*env)->GetLongField (env, thiz, fieldID)
# define SET_CUSTOM_DATA(env, thiz, fieldID, data) (*env)->SetLongField (env, thiz, fieldID, (jlong)(jint)data)
#endif

//#define SHOW_FPS_COUNTER

#define LOG_TAG "csi-display"

#define WIDTH 1920
#define HEIGHT 1200
#define BPP 3
#define VIDEO_DEV "/dev/video0"
#define NUM_BUFFERS 4
#define NUM_PLANES 1

#define GPIO_CHIP "/dev/gpiochip1"
#define GPIO_PIN 31

#define UART_DEV "/dev/ttyS2"
#define UART_BAUD_RATE B115200

struct buffer {
    void *start;
    size_t length;
};

typedef struct _CustomData
{
  jobject app;
  GstElement *pipeline;
  GstElement *appsrc;
  GMainContext *context;
  GMainLoop *main_loop;
  gboolean initialized;
  gboolean playing;
  GstElement *video_sink;
  ANativeWindow *native_window;

  int video_fd;
  int uart_fd;
  struct buffer *buffers;
  struct v4l2_requestbuffers req;
} CustomData;

static pthread_t gst_app_thread;
static pthread_t feeder_thread;
static pthread_key_t current_jni_env;
static JavaVM *java_vm;
static jfieldID custom_data_field_id;
static jmethodID set_message_method_id;
static jmethodID on_gstreamer_initialized_method_id;

static enum {
    START_FEED,
    STOP_FEED,
    EXIT_FEED
} feeder_state;

struct __attribute__((__packed__)) uart_data {
    uint8_t start;
    uint16_t x;
    uint16_t y;
    uint8_t slot;
    uint8_t press;
    uint8_t stop;
};

static JNIEnv *
attach_current_thread (void)
{
  JNIEnv *env;
  JavaVMAttachArgs args;

  args.version = JNI_VERSION_1_4;
  args.name = NULL;
  args.group = NULL;

  if ((*java_vm)->AttachCurrentThread (java_vm, &env, &args) < 0)
    return NULL;

  return env;
}

static void
detach_current_thread (void *env)
{
  (*java_vm)->DetachCurrentThread (java_vm);
}

static JNIEnv *
get_jni_env (void)
{
  JNIEnv *env;

  if ((env = pthread_getspecific (current_jni_env)) == NULL) {
    env = attach_current_thread ();
    pthread_setspecific (current_jni_env, env);
  }

  return env;
}

static void
error_cb (GstBus * bus, GstMessage * msg, CustomData * data)
{
  GError *err;
  gchar *debug_info;

  gst_message_parse_error (msg, &err, &debug_info);
  g_clear_error (&err);
  g_free (debug_info);
	__android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
			    "Error received from element %s: %s",
			    GST_OBJECT_NAME (msg->src), err->message);
  gst_element_set_state (data->pipeline, GST_STATE_NULL);
}

static void
state_changed_cb (GstBus * bus, GstMessage * msg, CustomData * data)
{
  GstState old_state, new_state, pending_state;
  gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);

  if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data->pipeline)) {
    g_print("State changed to %s",  gst_element_state_get_name (new_state));
  }
}

static void
check_initialization_complete (CustomData * data)
{
  JNIEnv *env = get_jni_env ();
  if (!data->initialized && data->native_window && data->main_loop) {
    GST_DEBUG
        ("Initialization complete, notifying application. native_window:%p main_loop:%p",
        data->native_window, data->main_loop);

    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (data->video_sink),
        (guintptr) data->native_window);

    (*env)->CallVoidMethod (env, data->app, on_gstreamer_initialized_method_id);
    if ((*env)->ExceptionCheck (env)) {
      __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,"Failed to call Java method");
      (*env)->ExceptionClear (env);
    }
    data->initialized = TRUE;
  }
}

static int setup_video_node(CustomData *data)
{
    struct v4l2_format format;
    struct v4l2_pix_format_mplane *mpix;
    enum v4l2_buf_type type;
    struct v4l2_requestbuffers *req = &data->req;
    int rc;

    data->video_fd = open(VIDEO_DEV, O_RDWR);
    if (data->video_fd < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,"Opening video device failed");
        return -EIO;
    }

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    mpix = &format.fmt.pix_mp;
    mpix->width = WIDTH;
    mpix->height = HEIGHT;
    mpix->pixelformat = V4L2_PIX_FMT_BGR24;
    mpix->field = V4L2_FIELD_NONE;
    mpix->num_planes = NUM_PLANES;
    mpix->plane_fmt[0].bytesperline = WIDTH * BPP;
    mpix->plane_fmt[0].sizeimage = WIDTH * HEIGHT * BPP;

    rc = ioctl(data->video_fd, VIDIOC_S_FMT, &format);
    if (rc < 0) {
	    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Setting format failed, rc %d, errno %d", rc, errno);
        goto close_node;
    }

    req->count = NUM_BUFFERS;
    req->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req->memory = V4L2_MEMORY_MMAP;

    rc = ioctl(data->video_fd, VIDIOC_REQBUFS, req);
    if (rc < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Buffer request failed, rc %d, errno %d", rc, errno);
	    goto close_node;
    }

    data->buffers = calloc(req->count, sizeof(*data->buffers));
    for (int i = 0; i < req->count; ++i) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes;

        memset(&buf, 0, sizeof buf);
        memset(&planes, 0, sizeof planes);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = NUM_PLANES;
        buf.m.planes = &planes;

        rc = ioctl(data->video_fd, VIDIOC_QUERYBUF, &buf);
        if (rc < 0) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                "Querying Buffer failed, rc %d, errno %d", rc, errno);
            goto close_node;
        }

        data->buffers[i].length = planes.length;
        data->buffers[i].start = mmap(NULL, planes.length,
                                      PROT_READ | PROT_WRITE, MAP_SHARED,
                                      data->video_fd, planes.m.mem_offset);
        if (data->buffers[i].start == MAP_FAILED) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                "Mapping buffer failed, errno %d, length: %zu, offset: %zu",
                                errno, planes.length, planes.m.mem_offset);
            goto close_node;
        }
    }

    for (int i = 0; i < req->count; ++i) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes;

        memset(&buf, 0, sizeof buf);
        memset(&planes, 0, sizeof planes);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = NUM_PLANES;
        buf.m.planes = &planes;

        rc = ioctl(data->video_fd, VIDIOC_QBUF, &buf);
        if (rc < 0) {
            __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,
                                "Queueing Buffer failed, rc %d, errno %d", rc, errno);
            goto close_node;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    rc = ioctl(data->video_fd, VIDIOC_STREAMON, &type);
    if (rc < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Start Capture failed, rc %d, errno %d", rc, errno);
	goto close_node;
    }

    return 0;

close_node:
	close(data->video_fd);
	return rc;
}

static void dispose_video_node(CustomData *data)
{
    for (int i = 0; i < data->req.count; ++i) {
        munmap(data->buffers[i].start, data->buffers[i].length);
    }

    free(data->buffers);
    close(data->video_fd);
}

static gboolean push_frame (CustomData *data)
{
    struct v4l2_plane planes;
    GstFlowReturn ret = GST_FLOW_OK;
    GstBuffer *buffer;
    struct v4l2_buffer buf;

    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = NUM_PLANES;
    buf.m.planes = &planes;
    if (ioctl(data->video_fd, VIDIOC_DQBUF, &buf) < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Dequeue buffer FAILED, errno %d", errno);
        return FALSE;
    }

    buffer = gst_buffer_new_allocate(NULL, planes.length, NULL);
    gst_buffer_fill(buffer, 0, data->buffers[buf.index].start, planes.length);
    g_signal_emit_by_name(data->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ioctl(data->video_fd, VIDIOC_QBUF, &buf) < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Queueing Buffer, errno %d", errno);
        return FALSE;
    }

    if (ret != GST_FLOW_OK) {
        return FALSE;
    }

    return TRUE;
}

static void *feeding_loop(void *data)
{
    JNIEnv *env = attach_current_thread();

    while (TRUE) {
        if (feeder_state == EXIT_FEED)
            break;

        if (feeder_state == START_FEED)
            push_frame(data);
    }

    detach_current_thread(env);

    return NULL;
}

static void start_feed (GstElement *source, guint size, CustomData *data)
{
    feeder_state = START_FEED;
}

static void stop_feed (GstElement *source, CustomData *data)
{
    feeder_state = STOP_FEED;
}

static void create_feeder_thread(CustomData *data)
{
    feeder_state = STOP_FEED;
    pthread_create(&feeder_thread, NULL, feeding_loop, data);
}

static void destroy_feeder_thread()
{
    feeder_state = EXIT_FEED;
    pthread_join(feeder_thread, NULL);
}

static void *app_function (void *userdata)
{
    CustomData *data = (CustomData *) userdata;
    GSource *bus_source;
    GstBus *bus;
    GError *error = NULL;

    if (setup_video_node(data)) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,"Video node setup FAILED");
        return NULL;
    }

    data->context = g_main_context_new ();
    g_main_context_push_thread_default (data->context);

    data->pipeline = gst_parse_launch ("appsrc name=video-source !"
                                       "video/x-raw,width=1920,height=1200,format=BGR,framerate=60/1 !"
                                       "videoconvert ! queue !"
#ifdef SHOW_FPS_COUNTER
                                       "fpsdisplaysink video-sink=glimagesink",
#else
                                       "glimagesink",
#endif
                                       &error);
    if (error) {
        g_clear_error (&error);
	    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
				"Unable to build pipeline: %s",
				error->message);
        goto dispose_node;
    }

    gst_element_set_state (data->pipeline, GST_STATE_READY);

    data->video_sink =
            gst_bin_get_by_interface (GST_BIN (data->pipeline),
                                      GST_TYPE_VIDEO_OVERLAY);
    if (!data->video_sink) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,"Could not retrieve video sink");
        goto dispose_node;
    }

    bus = gst_element_get_bus (data->pipeline);
    bus_source = gst_bus_create_watch (bus);
    g_source_set_callback (bus_source, (GSourceFunc) gst_bus_async_signal_func,
                           NULL, NULL);
    g_source_attach (bus_source, data->context);
    g_source_unref (bus_source);
    g_signal_connect (G_OBJECT (bus), "message::error", (GCallback) error_cb,
                      data);
    g_signal_connect (G_OBJECT (bus), "message::state-changed",
                      (GCallback) state_changed_cb, data);
    gst_object_unref (bus);

    data->appsrc = gst_bin_get_by_name(GST_BIN(data->pipeline), "video-source");
    g_object_set(G_OBJECT(data->appsrc), "is-live", TRUE,
		 "do-timestamp", TRUE, NULL);
    g_signal_connect (data->appsrc, "need-data", G_CALLBACK (start_feed), data);
    g_signal_connect (data->appsrc, "enough-data", G_CALLBACK (stop_feed), data);

    create_feeder_thread(data);

    gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
    data->playing = true;

    data->main_loop = g_main_loop_new (data->context, FALSE);
    check_initialization_complete (data);

    g_main_loop_run (data->main_loop);
    data->playing = false;

    g_main_loop_unref (data->main_loop);
    data->main_loop = NULL;

    destroy_feeder_thread();

    g_main_context_pop_thread_default (data->context);
    g_main_context_unref (data->context);
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
    gst_object_unref (data->video_sink);
    gst_object_unref (data->pipeline);

dispose_node:
    dispose_video_node(data);

    return NULL;
}

static void
gst_native_init (JNIEnv * env, jobject thiz)
{
  CustomData *data = g_new0 (CustomData, 1);

  SET_CUSTOM_DATA (env, thiz, custom_data_field_id, data);
  GST_DEBUG_CATEGORY_INIT (debug_category, LOG_TAG, 0, LOG_TAG);
  gst_debug_set_threshold_for_name (LOG_TAG, GST_LEVEL_DEBUG);
  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
		      "Created CustomData at %p", data);
  data->app = (*env)->NewGlobalRef (env, thiz);
  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
		      "Created GlobalRef for app object at %p", data->app);

  pthread_create (&gst_app_thread, NULL, &app_function, data);
}

static void
gst_native_finalize (JNIEnv * env, jobject thiz)
{
  CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
  if (!data)
    return;

  g_main_loop_quit (data->main_loop);
  pthread_join (gst_app_thread, NULL);
  (*env)->DeleteGlobalRef (env, data->app);
  g_free (data);
  SET_CUSTOM_DATA (env, thiz, custom_data_field_id, NULL);
}

static void
gst_native_play (JNIEnv * env, jobject thiz)
{
  CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
  if (!data)
    return;

  if (data->playing)
      return;

  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "PLAY");

  create_feeder_thread(data);

  gst_element_set_state (data->pipeline, GST_STATE_PLAYING);
  data->playing = true;
}

static void
gst_native_pause (JNIEnv * env, jobject thiz)
{
  CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
  if (!data)
    return;

  if (!data->playing)
      return;

  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "PAUSE");

  destroy_feeder_thread();

  gst_element_set_state (data->pipeline, GST_STATE_PAUSED);
  data->playing = false;
}

static jboolean
gst_native_class_init (JNIEnv * env, jclass klass)
{
  custom_data_field_id =
      (*env)->GetFieldID (env, klass, "nativeCustomData", "J");
  on_gstreamer_initialized_method_id =
      (*env)->GetMethodID (env, klass, "onGStreamerInitialized", "()V");

  if (!custom_data_field_id || !set_message_method_id
      || !on_gstreamer_initialized_method_id) {

    __android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
        "The calling class does not implement all necessary interface methods");
    return JNI_FALSE;
  }
  return JNI_TRUE;
}

static void
gst_native_surface_init (JNIEnv * env, jobject thiz, jobject surface)
{
  CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
  if (!data)
    return;
  ANativeWindow *new_native_window = ANativeWindow_fromSurface (env, surface);

  if (data->native_window) {
    ANativeWindow_release (data->native_window);
    if (data->native_window == new_native_window) {
      if (data->video_sink) {
        gst_video_overlay_expose (GST_VIDEO_OVERLAY (data->video_sink));
        gst_video_overlay_expose (GST_VIDEO_OVERLAY (data->video_sink));
      }
      return;
    } else {
      data->initialized = FALSE;
    }
  }
  data->native_window = new_native_window;

  check_initialization_complete (data);
}

static void
gst_native_surface_finalize (JNIEnv * env, jobject thiz)
{
  CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
  if (!data)
    return;

  if (data->video_sink) {
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (data->video_sink),
        (guintptr) NULL);
    gst_element_set_state (data->pipeline, GST_STATE_READY);
  }

  ANativeWindow_release (data->native_window);
  data->native_window = NULL;
  data->initialized = FALSE;
}

static void gst_gpio_set_pin(JNIEnv *env, jobject thiz, jboolean val)
{
	int fd;
	struct gpiohandle_request req;
	struct gpiohandle_data data;

	fd = open(GPIO_CHIP, O_RDONLY);
	if (fd < 0) {
		__android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
				     "Failed to open " GPIO_CHIP);
		return;
	}

	req.lineoffsets[0] = GPIO_PIN;
	req.flags = GPIOHANDLE_REQUEST_OUTPUT;
	req.lines = 1;
	data.values[0] = (uint8_t )val;

	if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
		__android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
				     "Failed to issue GET LINEHANDLE IOCTL");
		goto err_close;
	}

	if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
		__android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
				     "Failed to set line value");
        goto err_close;
	}

/*
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                        "Pin %d set to %s", GPIO_PIN,
                        val ? "HIGH" : "LOW");
*/

err_close:
	close(req.fd);
	close(fd);
}

static jboolean gst_setup_uart(JNIEnv *env, jobject thiz, jboolean do_open)
{
    CustomData *data = GET_CUSTOM_DATA(env, thiz, custom_data_field_id);

    if (!data) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Invalid data pointer");
        return JNI_FALSE;
    }

    if (do_open) {
        if (data->uart_fd > 0) {
            __android_log_print(ANDROID_LOG_WARN, LOG_TAG,
                                "UART already opened");
            return JNI_TRUE;
        }

        data->uart_fd = open(UART_DEV, O_WRONLY | O_NOCTTY | O_NDELAY);
        if (data->uart_fd < 0) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                "Failed to open UART device");
            return JNI_FALSE;
        }
    } else {
        if (data->uart_fd >= 0) {
            close(data->uart_fd);
            data->uart_fd = -1;
        }
    }

/*
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG,"%s %s",
                        do_open ? "Opened" : "Closed", UART_DEV);
*/

    return JNI_TRUE;
}

static jboolean gst_send_coordinates(JNIEnv *env, jobject thiz, jint x, jint y, jint slot, jint press)
{
    CustomData *data = GET_CUSTOM_DATA (env, thiz, custom_data_field_id);
    struct uart_data uart_data = { 0 };

    if (!data) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            "Invalid data pointer");
        return JNI_FALSE;
    }

    if (data->uart_fd < 0) {
        __android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
                             "Failed to open UART device");
        return JNI_FALSE;
    }

    uart_data.start = 0xAA;
    uart_data.x = (uint16_t)x;
    uart_data.y = (uint16_t)y;
    uart_data.slot = (uint8_t)slot;
    uart_data.press = (uint8_t)press;
    uart_data.stop = 0xBB;

    if (write(data->uart_fd, &uart_data, sizeof(uart_data)) < sizeof(uart_data)) {
        __android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
                             "Failed to write to UART device");
        return JNI_FALSE;
    }

/*
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                        "Written x %d, y %d, slot %d, press %d",
                        x, y, slot, press);
*/

    return JNI_TRUE;
}

/* List of implemented native methods */
static JNINativeMethod native_methods[] = {
  {"nativeInit", "()V", (void *) gst_native_init},
  {"nativeFinalize", "()V", (void *) gst_native_finalize},
  {"nativePlay", "()V", (void *) gst_native_play},
  {"nativePause", "()V", (void *) gst_native_pause},
  {"nativeSurfaceInit", "(Ljava/lang/Object;)V",
      (void *) gst_native_surface_init},
  {"nativeSurfaceFinalize", "()V", (void *) gst_native_surface_finalize},
  {"nativeClassInit", "()Z", (void *) gst_native_class_init},
  {"nativeSetPin", "(Z)V", (void *) gst_gpio_set_pin},
  {"nativeSetupUART", "(Z)Z", (void *) gst_setup_uart },
  {"nativeWriteCoordinates", "(IIII)Z", (void *) gst_send_coordinates }
};

/* Library initializer */
jint
JNI_OnLoad (JavaVM * vm, void *reserved)
{
  JNIEnv *env = NULL;

  java_vm = vm;

  if ((*vm)->GetEnv (vm, (void **) &env, JNI_VERSION_1_4) != JNI_OK) {
    __android_log_print (ANDROID_LOG_ERROR, LOG_TAG,
        "Could not retrieve JNIEnv");
    return 0;
  }
  jclass klass = (*env)->FindClass (env,
      "org/freedesktop/gstreamer/tutorials/tutorial_3/Tutorial3");
  (*env)->RegisterNatives (env, klass, native_methods,
      G_N_ELEMENTS (native_methods));

  pthread_key_create (&current_jni_env, detach_current_thread);

  return JNI_VERSION_1_4;
}
