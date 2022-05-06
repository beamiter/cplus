#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "discretized_dynamics.h"

template <typename T> struct ADVecotor {};

struct Euler : Explicit {};

struct RK3 : Explicit {};

struct RK4 : Explicit {};

#endif
