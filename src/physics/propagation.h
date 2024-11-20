#pragma once

#ifndef PROPAGATION_H
#define PROPAGATION_H

#include "kepler.h"
#include "../utils/darray.h"

PhysicalState apply_force_to(PhysicalState rv, DVector3 force, double delta_t);

#endif
