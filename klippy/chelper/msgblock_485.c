#include "msgblock_485.h"
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset

#define POLY 0x07   // CRC-8: x^8 + x^2 + x^1 + 1
uint8_t
msgblock_485_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Verify a buffer starts with a valid bus message
// head(1byte) + addr(1byte) + msglen(1byte) + data(msglen - 1 bytes) + crc8(1byte)
int
msgblock_485_check(uint8_t *need_sync, uint8_t *buf, int buf_len)
{
    uint8_t *head = buf;
    int check_len = buf_len;
    if (buf_len < MESSAGE_BUF_MIN) {
        errorf("buf_len = 0x%x", buf_len);
        for (int i = 0; i < buf_len; i++) {
            errorf("buf[%d] = 0x%x", i, buf[i]);
        }
        // Need more data
        return 0;
    }
    if (*need_sync) {
        goto error;
    }
    uint8_t msghead = buf[MESSAGE_485_POS_HEAD];
    if (msghead != MESSAGE_485_HEAD) {
        errorf("msghead = 0x%x, buf_len = 0x%x", msghead, buf_len);
        for (int i = 0; i < buf_len; i++) {
            errorf("buf[%d] = 0x%x", i, buf[i]);
        }
        goto error;
    }
    uint8_t msglen = buf[MESSAGE_485_POS_LEN];
    if (msglen < MESSAGE_485_MIN || msglen > MESSAGE_485_MAX) {
        head += 1;
        check_len -= 1;
        errorf("msglen = 0x%x, buf_len = 0x%x", msglen, buf_len);
        for (int i = 0; i < buf_len; i++) {
            errorf("buf[%d] = 0x%x", i, buf[i]);
        }
        goto error;
    }
    if (buf_len < msglen + MESSAGE_485_SIZE_OUTSIDE_DATA) {
        errorf("buf_len = 0x%x, msglen = 0x%x", buf_len, msglen);
        for (int i = 0; i < buf_len; i++) {
            errorf("buf[%d] = 0x%x", i, buf[i]);
        }
        // Need more data
        return 0;
    }
    uint8_t *datacrc = buf + MESSAGE_485_POS_LEN;
    uint32_t crclen = msglen;
    uint8_t msgcrc8 = datacrc[msglen];
    uint8_t crc = msgblock_485_crc8(datacrc, crclen);
    if (crc != msgcrc8) {
        head += 1;
        check_len -= 1;
        errorf("crc = 0x%x, msgcrc8 = 0x%x, buf_len = 0x%x", crc, msgcrc8, buf_len);
        // for (int i = 0; i < buf_len; i++) {
        //     errorf("buf[%d] = 0x%x", i, buf[i]);
        // }
        goto error;
    }
    return msglen + MESSAGE_485_SIZE_OUTSIDE_DATA;

error: ;
    // Discard bytes until next HEAD found
    uint8_t *next_head = memchr(head, MESSAGE_485_HEAD, check_len);
    // errorf("next_head = %p, head = %p, buf = %p", next_head, head, buf);
    errorf("Discard bytes until next HEAD found");
    for (int i = 0; i < buf_len; i++) {
        errorf("buf[%d] = 0x%x", i, buf[i]);
    }
    if (next_head) {
        *need_sync = 0;
        errorf("-(next_head - buf) = %d", -(next_head - buf));
        return -(next_head - buf);
    }

    errorf("-buf_len = %d", -buf_len);
    *need_sync = 1;
    return -buf_len;
}
