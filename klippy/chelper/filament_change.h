/*
 * @Description : 
 * @Author      : Yufeng Zhang
 * @Date: 2024-04-23 10:39:48
 * @LastEditTime: 2024-04-23 13:43:22
 */
#include <stdint.h>

typedef struct {
    uint8_t r, g, b;
} rgb_t;

int get_flushing_volume(const rgb_t source, const rgb_t target);

