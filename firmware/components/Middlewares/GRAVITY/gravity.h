#ifndef __GRAVITY_H_
#define __GRAVITY_H_

#include <stdbool.h>
typedef struct{
    float gx;
    float gy;
    bool  valid;
}gravity_xy_t;

void gravity_init(void);

void gravity_set(float gx,float gy);

gravity_xy_t gravity_get(void);

bool gravity_is_valid(void);


#endif // !__GRAVITY_H_