// Serial port command queuing
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// This goal of this code is to handle low-level serial port
// communications with a microcontroller (mcu).  This code is written
// in C (instead of python) to reduce communication latencies and to
// reduce scheduling jitter.  The code queues messages to be
// transmitted, schedules transmission of commands at specified mcu
// clock times, prioritizes commands, and handles retransmissions.  A
// background thread is launched to do this work and minimize latency.

#include <math.h> // fabs
#include <pthread.h> // pthread_mutex_lock
#include <stddef.h> // offsetof
#include <stdint.h> // uint64_t
#include <stdio.h> // snprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <termios.h> // tcflush
#include <unistd.h> // pipe
#include "compiler.h" // __visible
#include "msgblock_485.h"
#include "pollreactor.h" // pollreactor_alloc
#include "pyhelper.h" // errorf
#include "serial_485_queue.h" // struct queue_485_message

#define SQPF_SERIAL 0
#define SQPF_PIPE   1
#define SQPF_NUM    2

#define SQPT_COMMAND    0
#define SQPT_NUM        1

#define SQT_485 '4'
#define SQT_DEBUGFILE 'f'

#define DEBUG_QUEUE_SENT 100
#define DEBUG_QUEUE_RECEIVE 100

#define PR_NOW   0.
#define PR_NEVER 9999999999999999.

// Allocate a 'struct queue_485_message' object
static struct queue_485_message *
message_alloc(void)
{
    struct queue_485_message *qm = malloc(sizeof(*qm));
    memset(qm, 0, sizeof(*qm));
    return qm;
}

// Allocate a queue_485_message and fill it with the specified data
static struct queue_485_message *
message_fill(uint8_t *data, int len)
{
    struct queue_485_message *qm = message_alloc();
    memcpy(qm->msg, data, len);
    qm->len = len;
    return qm;
}

// Free the storage from a previous message_alloc() call
static void
message_free(struct queue_485_message **qm)
{
    if (*qm != NULL) {
        free(*qm);
        *qm = NULL;
    }
}

// Free all the messages on a queue
static void
message_queue_free(struct list_head *root)
{
    while (!list_empty(root)) {
        struct queue_485_message *qm = list_first_entry(
            root, struct queue_485_message, node);
        list_del(&qm->node);
        message_free(&qm);
    }
}

// Create a series of empty messages and add them to a list
static void
debug_queue_alloc(struct list_head *root, int count)
{
    int i;
    for (i=0; i<count; i++) {
        struct queue_485_message *qm = message_alloc();
        list_add_head(&qm->node, root);
    }
}

// Copy a message to a debug queue and free old debug messages
static void
debug_queue_add(struct list_head *root, struct queue_485_message *qm)
{
    list_add_tail(&qm->node, root);
    struct queue_485_message *old = list_first_entry(
        root, struct queue_485_message, node);
    list_del(&old->node);
    message_free(&old);
}

// Wake up the receiver thread if it is waiting
static void
check_wake_receive(struct serial_485_queue *sq)
{
    if (sq->receive_waiting) {
        sq->receive_waiting = 0;
        pthread_cond_signal(&sq->cond);
    }
}

// Write to the internal pipe to wake the background thread if in poll
static void
kick_bg_thread(struct serial_485_queue *sq)
{
    int ret = write(sq->pipe_fds[1], ".", 1);
    if (ret < 0)
        report_errno("pipe write", ret);
}

// Process a well formed input message
static void
handle_message(struct serial_485_queue *sq, double eventtime, int len)
{
    pthread_mutex_lock(&sq->lock);
    // pollreactor_update_timer(sq->pr, SQPT_COMMAND, PR_NOW);
    sq->bytes_read += len;

    sq->receive_queue = message_fill(sq->input_buf, len);

    check_wake_receive(sq);
    pthread_mutex_unlock(&sq->lock);
}

// Callback for input activity on the serial fd
static void
input_event(struct serial_485_queue *sq, double eventtime)
{
    int ret = read(sq->serial_fd, &sq->input_buf[sq->input_pos]
                    , sizeof(sq->input_buf) - sq->input_pos);
    if (ret <= 0) {
        if(ret < 0)
            report_errno("read", ret);
        else
            errorf("Got EOF when reading from device");
        pollreactor_do_exit(sq->pr);
        return;
    }
    sq->input_pos += ret;
    for (;;) {
        int len = 0;
        len = msgblock_485_check(&sq->need_sync, sq->input_buf, sq->input_pos);
        if (!len)
            // Need more data
            return;
        if (len > 0) {
            // Received a valid message
            handle_message(sq, eventtime, len);
        } else {
            // Skip bad data at beginning of input
            len = -len;
            pthread_mutex_lock(&sq->lock);
            sq->bytes_invalid += len;
            pthread_mutex_unlock(&sq->lock);
        }
        sq->input_pos -= len;
        if (sq->input_pos)
            memmove(sq->input_buf, &sq->input_buf[len], sq->input_pos);
    }
}

// Callback for input activity on the pipe fd (wakes command_event)
static void
kick_event(struct serial_485_queue *sq, double eventtime)
{
    char dummy[4096];
    int ret = read(sq->pipe_fds[0], dummy, sizeof(dummy));
    if (ret < 0)
        report_errno("pipe read", ret);
    pollreactor_update_timer(sq->pr, SQPT_COMMAND, PR_NOW);
}

static void
do_write(struct serial_485_queue *sq, void *buf, int buflen)
{
    int ret = write(sq->serial_fd, buf, buflen);
    if (ret < 0)
        report_errno("write", ret);
}

// Construct a block of data to be sent to the serial port
static int
build_and_send_command(struct serial_485_queue *sq, uint8_t *buf, double eventtime)
{
    int len = 0;
    if (sq->pending_queues == NULL) {
        return len;
    }
    len = MESSAGE_485_HEADER_SIZE;
    buf[MESSAGE_485_POS_HEAD] = MESSAGE_485_HEAD;
    memcpy(&buf[len], sq->pending_queues->msg, sq->pending_queues->len);
    len += sq->pending_queues->len;
    message_free(&sq->pending_queues);
    len += MESSAGE_485_TRAILER_SIZE;
    uint8_t crc8 = msgblock_485_crc8(&buf[MESSAGE_485_POS_LEN], buf[MESSAGE_485_POS_LEN]);
    buf[len - MESSAGE_485_TRAILER_CRC] = crc8;

    return len;
}

// Callback timer to send data to the serial port
static double
command_event(struct serial_485_queue *sq, double eventtime)
{
    pthread_mutex_lock(&sq->lock);
    uint8_t buf[512];
    memset(buf, 0, sizeof(buf));
    int buflen = 0;
    buflen = build_and_send_command(sq, &buf[buflen], eventtime);
    if (buflen) {
        // errorf("buf: %s, buf_len: %d", buf, buflen);
        // Write message blocks
        do_write(sq, buf, buflen);
        // errorf("buf: %s, buf_len: %d", buf, buflen);
        sq->bytes_write += buflen;
        buflen = 0;
    } else {
        pthread_mutex_unlock(&sq->lock);
        return PR_NEVER;
    }
    pthread_mutex_unlock(&sq->lock);
    // errorf("send finish");
    return PR_NEVER;
}

// Main background thread for reading/writing to serial port
static void *
background_thread(void *data)
{
    struct serial_485_queue *sq = data;
    nice(-20);
    pollreactor_run(sq->pr);

    pthread_mutex_lock(&sq->lock);
    check_wake_receive(sq);
    pthread_mutex_unlock(&sq->lock);

    return NULL;
}

// Create a new 'struct serial_485_queue' object
struct serial_485_queue * __visible
serial_485_queue_alloc(int serial_fd, char serial_fd_type)
{
    struct serial_485_queue *sq = malloc(sizeof(*sq));
    memset(sq, 0, sizeof(*sq));
    sq->serial_fd = serial_fd;
    sq->serial_fd_type = serial_fd_type;

    int ret = pipe(sq->pipe_fds);
    if (ret)
        goto fail;

    // Reactor setup
    sq->pr = pollreactor_alloc(SQPF_NUM, SQPT_NUM, sq);
    pollreactor_add_fd(sq->pr, SQPF_SERIAL, serial_fd, input_event
                       , serial_fd_type==SQT_DEBUGFILE);
    pollreactor_add_fd(sq->pr, SQPF_PIPE, sq->pipe_fds[0], kick_event, 0);
    pollreactor_add_timer(sq->pr, SQPT_COMMAND, command_event);
    fd_set_non_blocking(serial_fd);
    fd_set_non_blocking(sq->pipe_fds[0]);
    fd_set_non_blocking(sq->pipe_fds[1]);

    // Debugging
    // list_init(&sq->old_sent);
    // list_init(&sq->old_receive);
    // debug_queue_alloc(&sq->old_sent, DEBUG_QUEUE_SENT);
    // debug_queue_alloc(&sq->old_receive, DEBUG_QUEUE_RECEIVE);

    // Thread setup
    ret = pthread_mutex_init(&sq->lock, NULL);
    if (ret)
        goto fail;
    ret = pthread_cond_init(&sq->cond, NULL);
    if (ret)
        goto fail;
    ret = pthread_create(&sq->tid, NULL, background_thread, sq);
    if (ret)
        goto fail;

    return sq;

fail:
    report_errno("init", ret);
    return NULL;
}

// Request that the background thread exit
void __visible
serial_485_queue_exit(struct serial_485_queue *sq)
{
    pollreactor_do_exit(sq->pr);
    kick_bg_thread(sq);
    int ret = pthread_join(sq->tid, NULL);
    if (ret)
        report_errno("pthread_join", ret);
}

// Free all resources associated with a serial_485_queue
void __visible
serial_485_queue_free(struct serial_485_queue *sq)
{
    if (!sq)
        return;
    if (!pollreactor_is_exit(sq->pr))
        serial_485_queue_exit(sq);
    pthread_mutex_lock(&sq->lock);
    // message_queue_free(&sq->old_sent);
    // message_queue_free(&sq->old_receive);
    message_free(&sq->pending_queues);
    pthread_mutex_unlock(&sq->lock);
    pollreactor_free(sq->pr);
    free(sq);
}

// Schedule the transmission of a message on the serial port at a
// given time and priority.
void __visible
serial_485_queue_send(struct serial_485_queue *sq, uint8_t *msg, int len)
{
    pthread_mutex_lock(&sq->lock);
    sq->pending_queues = message_fill(msg, len);
    pthread_mutex_unlock(&sq->lock);
    kick_bg_thread(sq);
}

// Return a message read from the serial port (or wait for one if none
// available)
void __visible
serial_485_queue_pull(struct serial_485_queue *sq, struct pull_message *pqm)
{
    pthread_mutex_lock(&sq->lock);

    while (sq->receive_queue == NULL) {
        if (pollreactor_is_exit(sq->pr))
            goto exit;
        sq->receive_waiting = 1;
        int ret = pthread_cond_wait(&sq->cond, &sq->lock);
        if (ret)
            report_errno("pthread_cond_wait", ret);
    }

    memcpy(pqm->msg, sq->receive_queue->msg, sq->receive_queue->len);
    pqm->len = sq->receive_queue->len;
    message_free(&sq->receive_queue);

    pthread_mutex_unlock(&sq->lock);
    return;

exit:
    pqm->len = -1;
    pthread_mutex_unlock(&sq->lock);
}

// Return a string buffer containing statistics for the serial port
void __visible
serial_485_queue_get_stats(struct serial_485_queue *sq, char *buf, int len)
{
    struct serial_485_queue stats;
    pthread_mutex_lock(&sq->lock);
    memcpy(&stats, sq, sizeof(stats));
    pthread_mutex_unlock(&sq->lock);

    snprintf(buf, len, "bytes_write=%u bytes_read=%u bytes_invalid=%u"
             , stats.bytes_write, stats.bytes_read, stats.bytes_invalid);
}

// Extract old messages stored in the debug queues
int __visible
serial_485_queue_extract_old(struct serial_485_queue *sq, int sentq
                        , struct pull_message *q, int max)
{
    int count = sentq ? DEBUG_QUEUE_SENT : DEBUG_QUEUE_RECEIVE;
    struct list_head *rootp = sentq ? &sq->old_sent : &sq->old_receive;
    struct list_head replacement, current;
    list_init(&replacement);
    debug_queue_alloc(&replacement, count);
    list_init(&current);

    // Atomically replace existing debug list with new zero'd list
    pthread_mutex_lock(&sq->lock);
    list_join_tail(rootp, &current);
    list_init(rootp);
    list_join_tail(&replacement, rootp);
    pthread_mutex_unlock(&sq->lock);

    // Walk the debug list
    int pos = 0;
    while (!list_empty(&current)) {
        struct queue_485_message *qm = list_first_entry(
            &current, struct queue_485_message, node);
        if (qm->len && pos < max) {
            struct pull_message *pqm = q++;
            pos++;
            memcpy(pqm->msg, qm->msg, qm->len);
            pqm->len = qm->len;
        }
        list_del(&qm->node);
        message_free(&qm);
    }
    return pos;
}

