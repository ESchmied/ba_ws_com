#ifndef ODOMPARAM_H
#define ODOMPARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct{

   double x;
   double y;
   double z;
   double yaw;


   double v;


} OdomParam;

#ifdef __cplusplus
}
#endif

#endif //ODOMPARAM_H