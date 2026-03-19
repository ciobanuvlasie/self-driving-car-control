#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_print(int x, int y, const char *s);
void oled_update(void);

#endif
