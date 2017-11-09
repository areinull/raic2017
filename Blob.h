#pragma once

#ifndef MYSTRATEGY_BLOB_H
#define MYSTRATEGY_BLOB_H

#include "model/VehicleType.h"
#include "model/Move.h"
#include "Context.h"

class Blob {
public:
    explicit Blob(model::VehicleType type = model::VehicleType ::_UNKNOWN_);

    model::Move select(Context &ctx) const;
    model::Move move(Context &ctx, double x, double y) const;
    model::Move rndWalk(Context &ctx) const;
    void update(Context &ctx);
    bool isAlive() const { return alive_; }
    int getHpDeficit(Context &ctx) const;
    double getX() const { return x_; }
    double getY() const { return y_; }

private:
    model::VehicleType type_;
    double x_ = -1;
    double y_ = -1;
    bool alive_ = false;
};


#endif //MYSTRATEGY_BLOB_H
