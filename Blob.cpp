#define PI 3.14159265358979323846

#include "Blob.h"
#include "model/World.h"

using namespace model;

Blob::Blob(VehicleType type)
    : type_(type)
{
}

Move Blob::select(Context &ctx) const {
    Move m;
    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(0);
    m.setRight(ctx.world->getWidth());
    m.setBottom(ctx.world->getHeight());
    m.setVehicleType(type_);
    return m;
}

Move Blob::move(Context &, double x, double y) const {
    Move m;
    constexpr double dd_thr = 50. * 50.;
    const double dd = (x-x_)*(x-x_)+(y-y_)*(y-y_);
    if (dd > dd_thr) {
        m.setAction(ActionType::MOVE);
        m.setX(x - x_);
        m.setY(y - y_);
    } else {
        m.setAction(ActionType::ROTATE);
        m.setX((x + x_)/2.);
        m.setY((y + y_)/2.);
        m.setAngle((double)(rand()%5-2) * PI / 3.);
    }
    return m;
}

Move Blob::rndWalk(Context &ctx) const {
    Move m;
    m.setAction(ActionType::MOVE);
    m.setX(rand() % (int)ctx.world->getWidth() - x_);
    m.setY(rand() % (int)ctx.world->getHeight() - y_);
    return m;
}

void Blob::update(Context &ctx) {
    x_ = y_ = 0.;
    int cnt = 0;
    for (const auto &v: *ctx.vehicleById) {
        if (v.second.getPlayerId() == ctx.me->getId() && v.second.getType() == type_) {
            x_ += v.second.getX();
            y_ += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x_ /= cnt;
        y_ /= cnt;
        alive_ = true;
    } else {
        alive_ = false;
    }
}

int Blob::getHpDeficit(Context &ctx) const {
    int res = 0;
    for (const auto &v: *ctx.vehicleById) {
        if (v.second.getPlayerId() == ctx.me->getId() && v.second.getType() == type_) {
            res += v.second.getMaxDurability() - v.second.getDurability();
        }
    }
    return res;
}
