#include "V2d.h"

V2d operator*(double k, const V2d &rhs) {
    return {rhs.x*k, rhs.y*k};
}