#ifndef SERIAL_485_QUEUE_H
#define SERIAL_485_QUEUE_H

#include "msgblock_485.h"
#include "list.h" // struct list_node

#define BUFFER_MAX 512

struct queue_485_message {
    int len;
    uint8_t msg[BUFFER_MAX];
    struct list_node node;
};

struct pull_message {
    int len;
    uint8_t msg[BUFFER_MAX];
};

struct serial_485_queue {
    // Input reading
    struct pollreactor *pr;
    int serial_fd, serial_fd_type;
    int pipe_fds[2];
    uint8_t input_buf[4096];
    uint8_t need_sync;
    int input_pos;
    // Threading
    pthread_t tid;
    pthread_mutex_t lock; // protects variables below
    pthread_cond_t cond;
    int receive_waiting;
    // Pending transmission message queues
    struct queue_485_message *pending_queues;
    // Received messages
    // struct list_head receive_queue;
    struct queue_485_message *receive_queue;
    // Debugging
    struct list_head old_sent, old_receive;
    // Stats
    uint32_t bytes_write, bytes_read, bytes_invalid;
};

void serial_485_queue_send(struct serial_485_queue *sq, uint8_t *msg, int len);
void serial_485_queue_pull(struct serial_485_queue *sq, struct pull_message *pqm);
void serial_485_queue_get_stats(struct serial_485_queue *sq, char *buf, int len);
struct serial_485_queue * serial_485_queue_alloc(int serial_fd, char serial_fd_type);
void serial_485_queue_free(struct serial_485_queue *sq);
void serial_485_queue_exit(struct serial_485_queue *sq);

#endif /* SERIAL_485_QUEUE_H */
