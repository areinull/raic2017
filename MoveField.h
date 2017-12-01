#pragma once

#ifndef MYSTRATEGY_MOVEFIELD_H
#define MYSTRATEGY_MOVEFIELD_H

#include <vector>
#include "V2d.h"

struct Context;

class MoveField {
public:
    MoveField();

    void clearField();
    void addUnit(const V2d &p);
    void addObstacle(const V2d &p);
    std::vector<V2d> pathToNeg(const V2d &s) const;

private:
    static int clamp(int v);
    static int collapse(int x, int y);
    static std::pair<int, int> expand(int c);

    std::vector<int> pathToNeg(int s) const;


    static constexpr double gridStride_ = 4.;
    static constexpr int gridMax_ = 1024 / gridStride_;
    static constexpr int fireRange_ = (50-1)/gridStride_+1;
    static constexpr int nukeRange_ = (55-1)/gridStride_+1;

    int field_[gridMax_][gridMax_];

    friend std::ostream& operator<<(std::ostream&, const MoveField&);
};

std::ostream& operator<<(std::ostream&, const MoveField&);

#endif //MYSTRATEGY_MOVEFIELD_H
