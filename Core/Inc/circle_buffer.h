#ifndef _CIRCLE_BUF_H
#define _CIRCLE_BUF_H

#include <stdint.h>

typedef struct circle_buf {
	uint32_t r;
	uint32_t w;
	uint32_t len;
	uint8_t *buf;
}circle_buf, *p_circle_buf;

void circle_buf_init(p_circle_buf pCircleBuf, uint32_t len, uint8_t *buf);

int circle_buf_read(p_circle_buf pCircleBuf, uint8_t *pVal);

int circle_buf_write(p_circle_buf pCircleBuf, uint8_t val);

#endif /* _CIRCLE_BUF_H */
