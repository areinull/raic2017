#pragma once

#ifndef MYSTRATEGY_V2D_H
#define MYSTRATEGY_V2D_H


#include <cmath>

struct V2d {
    double x;
    double y;

    bool operator==(const V2d &o) const {
        return x == o.x && y == o.y;
    }

    bool operator==(V2d &&o) const {
        return x == o.x && y == o.y;
    }

    bool operator!=(const V2d &o) const {
        return !operator==(o);
    }

    bool operator!=(V2d &&o) const {
        return !operator==(o);
    }

    V2d operator+(const V2d &rhs) const {
        return {x+rhs.x, y+rhs.y};
    }

    V2d operator-(const V2d &rhs) const {
        return {x-rhs.x, y-rhs.y};
    }

    V2d& operator+=(const V2d &rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    V2d& operator-=(const V2d &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    V2d operator*(double k) const {
        return {x*k, y*k};
    }

    V2d operator/(double k) const {
        return {x/k, y/k};
    }

    V2d& operator*=(double k) {
        x *= k;
        y *= k;
        return *this;
    }

    V2d& operator/=(double k) {
        x /= k;
        y /= k;
        return *this;
    }

    double getNormSq() const {
        return x*x + y*y;
    }

    double getNorm() const {
        return std::sqrt(getNormSq());
    }

    V2d& normalize() {
        const double n = getNorm();
        x /= n;
        y /= n;
        return *this;
    }

    V2d unit() {
        return operator/(getNorm());
    }

    V2d rotate(double rad) const {
        const double s = std::sin(rad);
        const double c = std::cos(rad);
        return {x*c-y*s, x*s+y*c};
    }
};

V2d operator*(double k, const V2d &rhs);

#endif //MYSTRATEGY_V2D_H
