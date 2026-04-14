#ifndef MSGBLOCK_485_H
#define MSGBLOCK_485_H

#include <stdint.h>

/* head + addr + msglen + data, msglen >= 3, data = state + func + data + crc,  */
#define MESSAGE_BUF_MIN (6)
#define MESSAGE_485_MIN (3)
#define MESSAGE_485_MAX (255)
#define MESSAGE_485_HEAD (0xF7)
#define MESSAGE_485_POS_HEAD (0)
#define MESSAGE_485_POS_LEN (2)
#define MESSAGE_485_HEADER_SIZE (1)
#define MESSAGE_485_TRAILER_CRC (1) // crc
#define MESSAGE_485_TRAILER_SIZE (1) // crc
#define MAX_PENDING_BLOCKS_485 (1) // ask and answer interchangeably
#define MESSAGE_485_SIZE_OUTSIDE_DATA (3) // head + addr +len

uint8_t msgblock_485_crc8(const uint8_t *data, uint32_t len);
int msgblock_485_check(uint8_t *need_sync, uint8_t *buf, int buf_len);

#endif /* MSGBLOCK_485_H */
