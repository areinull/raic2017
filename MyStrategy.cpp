#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "MoveField.h"
#include "Clusterize.h"

using namespace model;

namespace {
    constexpr int LAND_GROUP = 1;
    constexpr int AIR_GROUP = 2;
    constexpr int NUKE_GROUP = 3;
    constexpr int ANTINUKE_GROUP = 4;
    constexpr int MAIN_GROUP = 4; // same as antinuke for now
    static VId vId = -1;
}

std::ostream& operator<<(std::ostream &s, VehicleType vt) {
    switch (vt) {
        case VehicleType::_UNKNOWN_:  s << "VehicleType::_UNKNOWN_"; break;
        case VehicleType::ARRV:       s << "VehicleType::ARRV"; break;
        case VehicleType::IFV:        s << "VehicleType::IFV"; break;
        case VehicleType::TANK:       s << "VehicleType::TANK"; break;
        case VehicleType::HELICOPTER: s << "VehicleType::HELICOPTER"; break;
        case VehicleType::FIGHTER:    s << "VehicleType::FIGHTER"; break;
        case VehicleType::_COUNT_:    s << "VehicleType::_COUNT_"; break;
    }
    return s;
}

MyStrategy::MyStrategy()
    : clist_(nullptr)
{
    ctx_.vehicleById = &vehicles_;
}

double MyStrategy::getWeatherVisibility(WeatherType w) const {
    switch (w) {
        case WeatherType::CLEAR: return ctx_.game->getClearWeatherVisionFactor();
        case WeatherType::CLOUD: return ctx_.game->getCloudWeatherVisionFactor();
        case WeatherType::RAIN:  return ctx_.game->getRainWeatherVisionFactor();
        default: return 0.;
    }
}

double MyStrategy::getTerrainVisibility(TerrainType t) const {
    switch (t) {
        case TerrainType::FOREST: return ctx_.game->getForestTerrainVisionFactor();
        case TerrainType::PLAIN:  return ctx_.game->getPlainTerrainVisionFactor();
        case TerrainType::SWAMP:  return ctx_.game->getSwampTerrainVisionFactor();
        default: return 0.;
    }
}

void MyStrategy::move(const Player& me, const World& world, const Game& game, Move& move) {
    initializeStrategy(game);
    initializeTick(me, world, game, move);

    if (me.getRemainingActionCooldownTicks() > 0) {
        return;
    }

    if (executeDelayedMove(move)) {
        return;
    }

    this->move();

    executeDelayedMove(move);
}

void MyStrategy::initializeStrategy(const model::Game &game) {
    static bool firstTime = true;
    if (firstTime) {
        firstTime = false;
        srand(game.getRandomSeed());
        slowestGroundSpeed_ = game.getTankSpeed() * game.getSwampTerrainSpeedFactor();
        slowestAirSpeed_ = game.getHelicopterSpeed() * game.getRainWeatherSpeedFactor();
        ctx_.vehicleById = &vehicles_;
    }
}

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

    if (clist_) {
        delete clist_;
        clist_ = nullptr;
    }

    for (auto &v: vehicles_) {
        v.second.prevPos = v.second.pos;
    }

    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()].v = vehicle;
        vehicles_[vehicle.getId()].lastUpdateTick = world.getTickIndex();
        vehicles_[vehicle.getId()].pos = {vehicle.getX(), vehicle.getY()};
        vehicles_[vehicle.getId()].prevPos = vehicles_[vehicle.getId()].pos;
        vehicles_[vehicle.getId()].isMine = vehicle.getPlayerId() == me.getId();
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
        } else {
            vehicles_[vehicleId].v = Vehicle(vehicles_[vehicleId].v, vehicleUpdate);
            vehicles_[vehicleId].lastUpdateTick = world.getTickIndex();
            vehicles_[vehicleId].pos = {vehicleUpdate.getX(), vehicleUpdate.getY()};
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
#ifdef MYDEBUG
    static unsigned callNum = 0;
    if (++callNum%20 == 0) {
        std::cout << '\n';
    }
    std::cout << moveQueue_.size() << ' ' << std::flush;
#endif
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.begin()->first <= ctx_.world->getTickIndex()) {
        const auto &elt = moveQueue_.begin()->second;
        move = elt.first;
        if (elt.second) {
            elt.second();
        }
        moveQueue_.erase(moveQueue_.begin());

        if (move.getAction() == ActionType::CLEAR_AND_SELECT && move.getGroup()) {
            if (onlyGroupSelected(move.getGroup())) {
                executeDelayedMove(move);
            }
        }
    }

    return true;
}

V2d MyStrategy::target(const V2d &c, bool acceptFacility, bool isMainForce) const {
    V2d t{0., 0.};

    // захват зданий
    while (acceptFacility) {
        int closestFacIdx = -1;
        double minDistSq = 1024.*1024.*4.;
        auto facilities = ctx_.world->getFacilities();

        bool gotFactoryToCapture = false;
        for (const auto &f: facilities) {
            if (f.getType() == FacilityType::VEHICLE_FACTORY &&
                f.getOwnerPlayerId() != ctx_.me->getId()) {
                gotFactoryToCapture = true;
                break;
            }
        }

        for (int i = 0, i_end = facilities.size(); i < i_end; ++i) {
            if (facilities[i].getOwnerPlayerId() == ctx_.me->getId() &&
                facilities[i].getCapturePoints() > 90.) {
                continue;
            }
            if (isMainForce && gotFactoryToCapture && facilities[i].getType() != FacilityType::VEHICLE_FACTORY) {
                continue;
            }

            const V2d facCenter{facilities[i].getLeft() + ctx_.game->getFacilityWidth()/2.,
                                facilities[i].getTop() + ctx_.game->getFacilityHeight()/2.};
            const double distSq = (c - facCenter).getNormSq();
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestFacIdx = i;
            }
        }
        if (closestFacIdx < 0)
            break;
        t.x = facilities[closestFacIdx].getLeft() + ctx_.game->getFacilityWidth()/2.;
        t.y = facilities[closestFacIdx].getTop() + ctx_.game->getFacilityHeight()/2.;
        return t;
    }

    // центр масс противника
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (!v.second.isMine) {
            t += v.second.pos;
            ++cnt;
        }
    }
    if (cnt) {
        t /= cnt;
    }

    // проверям, что в центре что-то есть
    constexpr double offset = 32.;
    const double xmin = clampX(t.x - offset),
                 xmax = clampX(t.x + offset),
                 ymin = clampY(t.y - offset),
                 ymax = clampY(t.y + offset);
    bool gotEnemyInside = false;
    for (const auto &vext: vehicles_) {
        if (!vext.second.isMine &&
            vext.second.pos.x > xmin &&
            vext.second.pos.x < xmax &&
            vext.second.pos.y > ymin &&
            vext.second.pos.y < ymax) {
            gotEnemyInside = true;
            break;
        }
    }

    // ищем ближайший юнит
    constexpr double minDistThresholdSq = 256.*256.;
    double minDistSq = std::numeric_limits<double>::max();
    VId closest = -1;
    for (const auto &vext: vehicles_) {
        if (vext.second.isMine)
            continue;
        const double distSq = vext.second.v.getSquaredDistanceTo(c.x, c.y);
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closest = vext.first;
        }
    }

    if ((!gotEnemyInside || minDistSq < minDistThresholdSq) && closest >= 0) {
        t = vehicles_.at(closest).pos;
    }
    return t;
}

void MyStrategy::nuke(const V2d &c, V2d &nukePos, VId &strikeUnit) {
    if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
        return;
    }

    const auto t = target(c, false);

    const V2d eDir = (t - c).unit();

    const auto quarterSpan = 32.;
    const auto pointA = c + eDir*quarterSpan;

    // choose strike vehicle near A
    VId vehicleId = -1;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &v: vehicles_) {
        if (!v.second.isMine)
            continue;

        const double visionCoeff = v.second.v.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.pos.x/32][(int)v.second.pos.y/32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.pos.x/32][(int)v.second.pos.y/32]);
        const double score = v.second.v.getSquaredDistanceTo(pointA.x, pointA.y) - v.second.v.getVisionRange()*visionCoeff*v.second.v.getVisionRange()*visionCoeff;
        if (score < best_score) {
            best_score = score;
            vehicleId = v.first;
        }
    }
    const auto &vext = vehicles_[vehicleId];

    // choose strike point
    const double strikeDistance = vext.v.getVisionRange()*0.9;
    V2d pointB = t;
    if (vext.v.getSquaredDistanceTo(t.x, t.y) > strikeDistance*strikeDistance) {
        const auto tDir = (t - vext.pos).unit();
        pointB = vext.pos + tDir*strikeDistance;
    }

    // estimate damage in strike zone
    double balanceDamage = 0.;
    for (const auto &v: vehicles_) {
        const auto d = v.second.v.getDistanceTo(pointB.x, pointB.y);
        if ( d > ctx_.game->getTacticalNuclearStrikeRadius())
            continue;
        const double dmg = ctx_.game->getMaxTacticalNuclearStrikeDamage() * (1 - d/ctx_.game->getTacticalNuclearStrikeRadius());
        if (v.second.isMine) {
            balanceDamage -= 3.*dmg;
        } else {
            balanceDamage += dmg;
        }
    }

    // strike!
    if (balanceDamage > 10.*ctx_.game->getMaxTacticalNuclearStrikeDamage()) {
        nukePos = pointB;
        strikeUnit = vehicleId;
    }

    // try closest enemy
    VId closestEnemy = -1;
    double minDistSq = 1024.*1024.*4.;
    for (const auto &v: vehicles_) {
        if (v.second.isMine)
            continue;
        const double d = v.second.v.getSquaredDistanceTo(c.x, c.y);
        if (d < minDistSq) {
            minDistSq = d;
            closestEnemy = v.first;
        }
    }
    if (closestEnemy < 0)
        return;

    pointB = vehicles_[closestEnemy].pos;

    // find closest friendly unit
    VId closestFriend = -1;
    minDistSq = 1024.*1024.;
    for (const auto &v: vehicles_) {
        if (!v.second.isMine || v.first == vId)
            continue;
        const double visionCoeff = v.second.v.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.pos.x/32][(int)v.second.pos.y/32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.pos.x/32][(int)v.second.pos.y/32]);
        const double d = v.second.v.getSquaredDistanceTo(pointB.x, pointB.y) -
                v.second.v.getSquaredVisionRange()*visionCoeff*visionCoeff;
        if (d < minDistSq) {
            minDistSq = d;
            closestFriend = v.first;
        }
    }
    if (closestFriend < 0)
        return;

    const double minDist = vehicles_[closestFriend].v.getDistanceTo(vehicles_[closestEnemy].v);
    const double visionCoeff = vehicles_[closestFriend].v.isAerial()?
                               getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)vehicles_[closestFriend].pos.x/32][(int)vehicles_[closestFriend].pos.y/32]):
                               getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)vehicles_[closestFriend].pos.x/32][(int)vehicles_[closestFriend].pos.y/32]);
    const double curVisionRange = vehicles_[closestFriend].v.getVisionRange()*visionCoeff;
    if (minDist > ctx_.game->getTacticalNuclearStrikeRadius() && minDist < curVisionRange*0.95) {
        nukePos = pointB;
        strikeUnit = closestFriend;
    }
}


void MyStrategy::move() {
    if (ctx_.world->getTickIndex() == 0) {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        queueMove(0, m);

        m.setAction(ActionType::ASSIGN);
        m.setGroup(ANTINUKE_GROUP);
        queueMove(0, m);
    }

    nukeStriker();

    const bool isStartup = startupFormation();

    if (!isStartup && antiNuke()) {
        return;
    }

    if (!isStartup) {
        mainForce();
        manageFacilities();
        if (ctx_.world->getTickIndex()%60 == 0) {
            manageClusters();
        }
    }
}

void MyStrategy::queueMove(int delay, const Move &m, std::function<void(void)> &&f) {
    moveQueue_.emplace(ctx_.world->getTickIndex() + delay, std::make_pair(m, f));
}

bool MyStrategy::mainForce() {
    enum class State {
        Idle,
        Scale,
        Move,
        NukeStrike,
        NukeStrikeWait,
        End
    };
    static State state{State::Idle};
    static V2d pos{0., 0.},
               t{0., 0.},
               d{0., 0.};
    constexpr double dd_max = 64.;
    constexpr int maxTickInaction = 60;
    static bool waitingMoveQueue = false;
    static int lastActionTick = 0;

    if (state == State::End)
        return false;

    V2d nukePos;
    VId strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0 && strikeUnit != vId) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (ctx_.world->getTickIndex() - lastActionTick < maxTickInaction &&
                v.second.isMine &&
                std::find(v.second.v.getGroups().begin(),
                          v.second.v.getGroups().end(),
                          MAIN_GROUP) != v.second.v.getGroups().end() &&
                v.second.hasMoved()) {
                // юниты ещё перемещаются
                return true;
            }
        }
    }
    lastActionTick = ctx_.world->getTickIndex();

    bool haveUnits;
    std::tie(haveUnits, pos) = getCenter(LAND_GROUP);
    if (!haveUnits) {
        state = State::End;
        return false;
    }

    switch (state) {
        case State::Idle: {
            // прилипание кластера
            {
                if (!clist_) {
                    clist_ = Clusterize::clusterize2(ctx_, 10.);
                }
                double xmin = ctx_.world->getWidth(),
                       xmax = 0.,
                       ymin = ctx_.world->getHeight(),
                       ymax = 0.;
                for (const auto &c: clist_->myClusters) {
                    if (c.isAir) {
                        continue;
                    }
                    bool gotMainGroup = false;
                    bool gotOnlyMainGroup = true;
                    for (auto vid: c.set) {
                        if (std::find(vehicles_[vid].v.getGroups().begin(),
                                      vehicles_[vid].v.getGroups().end(),
                                      MAIN_GROUP) != vehicles_[vid].v.getGroups().end()) {
                            gotMainGroup = true;
                            if (!gotOnlyMainGroup)
                                break;
                        } else {
                            gotOnlyMainGroup = false;
                        }
                    }
                    if (!gotMainGroup || gotOnlyMainGroup) {
                        continue;
                    }
                    for (auto vid: c.set) {
                        xmin = std::min(xmin, vehicles_[vid].pos.x);
                        xmax = std::max(xmax, vehicles_[vid].pos.x);
                        ymin = std::min(ymin, vehicles_[vid].pos.y);
                        ymax = std::max(ymax, vehicles_[vid].pos.y);
                    }
                }

                if (xmin < xmax && ymin < ymax &&
                    (vehicles_[vId].pos.x < xmin || vehicles_[vId].pos.x > xmax ||
                     vehicles_[vId].pos.y < ymin || vehicles_[vId].pos.y > ymax)) {
                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(xmin);
                    m.setTop(ymin);
                    m.setRight(xmax);
                    m.setBottom(ymax);
                    m.setVehicleType(VehicleType::IFV);
                    queueMove(0, m);

                    m.setAction(ActionType::ADD_TO_SELECTION);
                    m.setVehicleType(VehicleType::TANK);
                    queueMove(0, m);

                    m.setAction(ActionType::ASSIGN);
                    m.setGroup(MAIN_GROUP);
                    queueMove(0, m);

                    m.setGroup(LAND_GROUP);
                    queueMove(0, m);

                    //m.setGroup(ANTINUKE_GROUP);
                    //queueMove(0, m);

                    state = State::Scale;
                    break;
                }
            }

            // сжаться
            {
                double xmin = ctx_.game->getWorldWidth(),
                        xmax = 0.,
                        ymin = ctx_.game->getWorldHeight(),
                        ymax = 0.;
                for (const auto &vext: vehicles_) {
                    if (!vext.second.isMine ||
                        std::find(vext.second.v.getGroups().begin(),
                                  vext.second.v.getGroups().end(),
                                  MAIN_GROUP) == vext.second.v.getGroups().end()) {
                        continue;
                    }
                    xmin = std::min(xmin, vext.second.pos.x);
                    xmax = std::max(xmax, vext.second.pos.x);
                    ymin = std::min(ymin, vext.second.pos.y);
                    ymax = std::max(ymax, vext.second.pos.y);
                }
                const double ratio = (xmax - xmin) / (ymax - ymin);
                constexpr double min_ratio = 2.5;
                constexpr double max_span = 200.;
                if (ratio > min_ratio || ratio < 1. / min_ratio ||
                    xmax - xmin > max_span ||
                    ymax - ymin > max_span) {
                    state = State::Scale;
                    return mainForce();
                }
            }

            t = clamp4main(pos, target(pos, true, true));
            d = t - pos;
            double dd = d.getNorm();
            if (dd > dd_max) {
                const double k = dd_max / dd;
                d *= k;
                dd = dd_max;
            }
            state = State::Move;
            return mainForce();
        }

        case State::Scale: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(MAIN_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.x);
            m.setY(pos.y);
            m.setAngle(PI/4.);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            queueMove(60, m);

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(60, m);

            state = State::Idle;
        }
        break;

        case State::Move: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(d.x);
            m.setY(d.y);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            bool hasAir;
            V2d posAir;
            std::tie(hasAir, posAir) = getCenter(AIR_GROUP);
            if (hasAir) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(AIR_GROUP);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(pos.x - posAir.x + d.x);
                m.setY(pos.y - posAir.y + d.y);
                m.setMaxSpeed(slowestGroundSpeed_);
                queueMove(0, m);
            }

            state = State::Idle;
        }
        break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.x);
            m.setY(nukePos.y);
            m.setVehicleId(strikeUnit);
            queueMove(0, m, [&waitingMoveQueue](){ waitingMoveQueue = false; });
            waitingMoveQueue = true;

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(MAIN_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(0);
            m.setY(0);
            queueMove(0, m);

            state = State::NukeStrikeWait;
        }
        break;

        case State::NukeStrikeWait: {
            if (!waitingMoveQueue && ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
                state = State::Idle;
                return mainForce();
            }
        }
        break;

        case State::End:
            return false;
    }
    return true;
}

double MyStrategy::clampX(double x) const {
    return std::max(0., std::min(ctx_.world->getWidth(), x));
}

double MyStrategy::clampY(double y) const {
    return std::max(0., std::min(ctx_.world->getHeight(), y));
}

V2d MyStrategy::clamp4main(V2d c, V2d t) const {
    constexpr double offset = 5.;
    double xmin = ctx_.game->getWorldWidth(),
           xmax = 0.,
           ymin = ctx_.game->getWorldHeight(),
           ymax = 0.;
    for (const auto &vext: vehicles_) {
        if (!vext.second.isMine ||
            std::find(vext.second.v.getGroups().begin(),
                      vext.second.v.getGroups().end(),
                      MAIN_GROUP) == vext.second.v.getGroups().end()) {
            continue;
        }
        xmin = std::min(xmin, vext.second.pos.x);
        xmax = std::max(xmax, vext.second.pos.x);
        ymin = std::min(ymin, vext.second.pos.y);
        ymax = std::max(ymax, vext.second.pos.y);
    }
    const double dxmin = c.x - xmin,
                 dxmax = xmax - c.x,
                 dymin = c.y - ymin,
                 dymax = ymax - c.y;
    const double txmin = std::max(t.x - dxmin, offset) + dxmin,
                 tymin = std::max(t.y - dymin, offset) + dymin,
                 txmax = std::min(t.x + dxmax, ctx_.game->getWorldWidth()-offset) - dxmax,
                 tymax = std::min(t.y + dymax, ctx_.game->getWorldHeight()-offset) - dymax;
    return {std::max(txmin, std::min(txmax, t.x)),
            std::max(tymin, std::min(tymax, t.y))};
}

bool MyStrategy::antiNuke() {
    enum class State {
        Idle,
        Spread,
        Converge
    };
    static auto state = State::Idle;
    static int moveCnt = 0;
    static V2d nukePos{-1., -1.};

    switch (state) {
        case State::Idle: {
            if (ctx_.world->getOpponentPlayer().getNextNuclearStrikeTickIndex() < 0) {
                return false;
            }
            bool vehicleInRange = false;
            const double strikeX = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
            const double strikeY = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();
            for (const auto &v: vehicles_) {
                if (!v.second.isMine ||
                    std::find(v.second.v.getGroups().begin(),
                              v.second.v.getGroups().end(),
                              ANTINUKE_GROUP) == v.second.v.getGroups().end())
                    continue;
                const auto d = v.second.v.getSquaredDistanceTo(strikeX, strikeY);
                if (d < ctx_.game->getTacticalNuclearStrikeRadius()*ctx_.game->getTacticalNuclearStrikeRadius()) {
                    vehicleInRange = true;
                    break;
                }
            }
            if (vehicleInRange) {
                state = State::Spread;
                return antiNuke();
            } else {
                return false;
            }
        }

        case State::Spread: {
            if (ctx_.world->getOpponentPlayer().getNextNuclearStrikeTickIndex() > 0) {
                if (!moveCnt) {
                    nukePos.x = ctx_.world->getOpponentPlayer().getNextNuclearStrikeX();
                    nukePos.y = ctx_.world->getOpponentPlayer().getNextNuclearStrikeY();

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setGroup(ANTINUKE_GROUP);
                    queueMove(0, m);

                    m.setAction(ActionType::SCALE);
                    m.setX(nukePos.x);
                    m.setY(nukePos.y);
                    m.setFactor(10);
                    queueMove(0, m);
                }
                ++moveCnt;
            } else {
                state = State::Converge;
                return antiNuke();
            }
        }
        break;

        case State::Converge: {
            if (!moveCnt) {
                state = State::Idle;
                return false;
            }
            if (nukePos.x >= 0) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ANTINUKE_GROUP);
                queueMove(0, m);

                m.setAction(ActionType::SCALE);
                m.setX(nukePos.x);
                m.setY(nukePos.y);
                m.setFactor(0.1);
                queueMove(0, m);

                nukePos = {-1., -1.};
            }
            --moveCnt;
        }
        break;
    }

    return true;
}

bool MyStrategy::startupFormation() {
    enum class State {
        Init,
        XArrange,
        YArrange,
        Merge,
        Scale,
        Rotate,
        WaitEnd,
        End
    };
    static auto state{State::Init};

    static std::array<VehicleType, 3> grNumGround{VehicleType::TANK, VehicleType::IFV, VehicleType::ARRV};
    static std::map<VehicleType, int> type2idxGround{
            {VehicleType::TANK, 0},
            {VehicleType::IFV,  1},
            {VehicleType::ARRV, 2}
    };
    static std::array<V2d, 3> grPosGround;

    static std::array<VehicleType, 2> grNumAir{VehicleType::FIGHTER, VehicleType::HELICOPTER};
    static std::map<VehicleType, int> type2idxAir{
            {VehicleType::FIGHTER, 0},
            {VehicleType::HELICOPTER,  1},
    };
    static std::array<V2d, 2> grPosAir;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.isMine && v.first != vId && v.second.hasMoved()) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            grPosGround.fill(V2d{0., 0.});
            grPosAir.fill(V2d{0., 0.});

            for (const auto &v: vehicles_) {
                if (!v.second.isMine || v.first == vId)
                    continue;
                if (v.second.v.isAerial()) {
                    auto &pos = grPosAir[type2idxAir[v.second.v.getType()]];
                    if (v.second.v.getType() == VehicleType::FIGHTER)
                        pos += v.second.pos/99.;
                    else
                        pos += v.second.pos/100.;
                } else {
                    auto &pos = grPosGround[type2idxGround[v.second.v.getType()]];
                    pos += v.second.pos / 100.;
                }
            }

            {
                int firstGr = 0;
                double minX = grPosGround[0].x;
                for (int i: {1, 2}) {
                    if (grPosGround[i].x < minX) {
                        minX = grPosGround[i].x;
                        firstGr = i;
                    }
                }
                std::swap(grNumGround[0], grNumGround[firstGr]);
                std::swap(grPosGround[0], grPosGround[firstGr]);

                if (grPosGround[1].x > grPosGround[2].x) {
                    std::swap(grNumGround[1], grNumGround[2]);
                    std::swap(grPosGround[1], grPosGround[2]);
                }

                for (int i = 0; i < 3; ++i) {
                    type2idxGround[grNumGround[i]] = i;
                }
            }

            {
                if (grPosAir[0].x > grPosAir[1].x) {
                    std::swap(grNumAir[0], grNumAir[1]);
                    std::swap(grPosAir[0], grPosAir[1]);
                }

                for (int i = 0; i < 2; ++i) {
                    type2idxAir[grNumAir[i]] = i;
                }
            }

            state = State::XArrange;
            return startupFormation();
        }

        case State::XArrange: {
            const double xMid = std::max(grPosGround[1].x, 110.);
            const double xCenter[] = {xMid - 70., xMid, xMid + 70.};

            for (int i: {0, 1, 2})  {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNumGround[i]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(xCenter[i] - grPosGround[i].x);
                m.setY(0);
                queueMove(0, m);
                grPosGround[i].x = xCenter[i];
            }

            for (int i: {0, 1})  {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNumAir[i]);
                queueMove(0, m);

                if (grNumAir[i] == VehicleType::FIGHTER) {
                    m.setAction(ActionType::DESELECT);
                    m.setGroup(NUKE_GROUP);
                    queueMove(0, m);
                }

                m.setAction(ActionType::MOVE);
                m.setX(xCenter[i] - grPosAir[i].x);
                m.setY(0);
                queueMove(0, m);
                grPosAir[i].x = xCenter[i];
            }

            state = State::YArrange;
        }
            break;

        case State::YArrange: {
            double yLines[10] = {10.};
            for (unsigned i=1; i<10; ++i) {
                yLines[i] = yLines[i-1] + 15.;
            }
            const double yShift[] = {-5., 0., 5.};

            for (int i=0; i<3; ++i) {
                double yMax = 0.;
                double yMin = 1024.;
                for (const auto &v: vehicles_) {
                    if (v.second.isMine && v.second.v.getType() == grNumGround[i]) {
                        yMax = std::max(yMax, v.second.pos.y);
                        yMin = std::min(yMin, v.second.pos.y);
                    }
                }
                const double ySpan = yMax - yMin;
                const double yStride = ySpan/9.;
                for (int j=0; j<10; ++j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget > yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNumGround[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }

                for (int j=9; j>=0; --j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget < yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNumGround[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }
            }

            for (int i=0; i<2; ++i) {
                double yMax = 0.;
                double yMin = 1024.;
                for (const auto &v: vehicles_) {
                    if (v.second.isMine && v.second.v.getType() == grNumAir[i] && v.first != vId) {
                        yMax = std::max(yMax, v.second.pos.y);
                        yMin = std::min(yMin, v.second.pos.y);
                    }
                }
                const double ySpan = yMax - yMin;
                const double yStride = ySpan/9.;
                for (int j=0; j<10; ++j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget > yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNumAir[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }

                for (int j=9; j>=0; --j) {
                    const double yCur = yMin + j*yStride;
                    const double yTarget = yLines[j] + yShift[i];
                    if (yTarget < yCur)
                        break;

                    Move m;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(0);
                    m.setTop(yCur - 2.);
                    m.setRight(ctx_.world->getWidth());
                    m.setBottom(yCur + 2.);
                    m.setVehicleType(grNumAir[i]);
                    queueMove(0, m);

                    m.setAction(ActionType::MOVE);
                    m.setX(0);
                    m.setY(yTarget - yCur);
                    queueMove(0, m);
                }
            }

            state = State::Merge;
        }
            break;

        case State::Merge: {
            for (int i: {0, 2}) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNumGround[i]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(grPosGround[1].x - grPosGround[i].x);
                m.setY(0);
                queueMove(0, m);
            }
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNumAir[0]);
                queueMove(0, m);

                if (grNumAir[0] == VehicleType::FIGHTER) {
                    m.setAction(ActionType::DESELECT);
                    m.setGroup(NUKE_GROUP);
                    queueMove(0, m);
                }

                m.setAction(ActionType::MOVE);
                m.setX(grPosAir[1].x - grPosAir[0].x);
                m.setY(0);
                queueMove(0, m);
            }
            state = State::Scale;
        }
            break;

        case State::Scale: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            double xmin = ctx_.world->getWidth(),
                    xmax = 0.,
                    ymin = ctx_.world->getHeight(),
                    ymax = 0.;
            for (const auto &v: vehicles_) {
                if (v.second.isMine && v.first != vId) {
                    xmin = std::min(xmin, v.second.pos.x);
                    xmax = std::max(xmax, v.second.pos.x);
                    ymin = std::min(ymin, v.second.pos.y);
                    ymax = std::max(ymax, v.second.pos.y);
                }
            }

            m.setAction(ActionType::SCALE);
            m.setX((xmin+xmax)/2.);
            m.setY((ymin+ymax)/2.);
            m.setFactor(0.1);
            queueMove(0, m);

            //m.setAction(ActionType::ASSIGN);
            //m.setGroup(MAIN_GROUP);
            //queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(0);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(0);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setGroup(0);
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            state = State::Rotate;
        }
            break;

        case State::Rotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(MAIN_GROUP);
            queueMove(0, m);

            V2d c;
            std::tie(std::ignore, c) = getCenter(LAND_GROUP);

            m.setAction(ActionType::ROTATE);
            m.setX(c.x);
            m.setY(c.y);
            m.setAngle(PI/4.);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::WaitEnd;
        }
        break;

        case State::WaitEnd:
            state = State::End;
            return startupFormation();

        case State::End:
            return false;
    }
    return true;
}

std::pair<bool, V2d> MyStrategy::getCenter(int group) const {
    int cnt = 0;
    V2d res{0., 0.};
    for (const auto &v: vehicles_) {
        if (v.second.isMine &&
            std::find(v.second.v.getGroups().begin(),
                      v.second.v.getGroups().end(),
                      group) != v.second.v.getGroups().end()) {
            res += v.second.pos;
            ++cnt;
        }
    }
    if (!cnt) {
        return {false, res};
    }
    res /= cnt;
    return {true, res};
}

bool MyStrategy::nukeStriker() {
    enum class State {
        Init,
        Idle,
        Move,
        Nuke,
        End
    };

    static auto state = State::Init;

    if (state == State::End) {
        return false;
    }

    if (state != State::Init) {
        if (!vehicles_.count(vId)) {
            bool gotFighter = false;
            for (const auto &v: vehicles_) {
                if (v.second.isMine && v.second.v.getType() == VehicleType::FIGHTER) {
                    gotFighter = true;
                    break;
                }
            }
            if (gotFighter) {
                state = State::Init;
            } else {
                state = State::End;
            }
        } else if (vehicles_[vId].hasMoved()) {
            return true;
        }
    }

    switch (state) {
        case State::Init: {
            V2d pos_max{0., 0.};
            for (const auto &v: vehicles_) {
                if (!v.second.isMine || v.second.v.getType() != VehicleType::FIGHTER)
                    continue;
                if (v.second.pos.x >= pos_max.x && v.second.pos.y >= pos_max.y) {
                    pos_max = v.second.pos;
                    vId = v.first;
                }
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(pos_max.x - 2);
            m.setTop(pos_max.y - 2);
            m.setRight(pos_max.x + 2);
            m.setBottom(pos_max.y + 2);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::DISMISS);
            m.setGroup(MAIN_GROUP);
            queueMove(0, m);

            m.setGroup(AIR_GROUP);
            queueMove(0, m);

//            m.setGroup(ANTINUKE_GROUP);
//            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(32);
            m.setY(32);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::Idle: {
            MoveField f;
            for (const auto &v: vehicles_) {
                if (!v.second.isMine) {
                    f.addUnit(v.second.pos);
                } else if (v.second.v.isAerial() && v.first != vId) {
                    f.addObstacle(v.second.pos);
                }
            }
            f.addWeather(ctx_);

            auto path = f.pathToNeg(vehicles_[vId].pos, false);

            if (path.empty()) {
                if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
                    path = f.pathToNeg(vehicles_[vId].pos, true);
                    if (path.empty()) {
                        state = State::Nuke;
                        return nukeStriker();
                    }
                } else {
                    state = State::Nuke;
                    return nukeStriker();
                }
            }

            int dest_idx = 0;
            for (int i = path.size()-1; i > 0; --i) {
                if ((path[i] - vehicles_[vId].pos).getNormSq() < 100.*100. &&
                    f.segmentClear(vehicles_[vId].pos, path[i])) {
                    dest_idx = i;
                    break;
                }
            }
            const auto dest = path[dest_idx];

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(NUKE_GROUP);
            queueMove(0, m);

            for (int gr: vehicles_[vId].v.getGroups()) {
                if (gr != NUKE_GROUP) {
                    m.setAction(ActionType::DISMISS);
                    m.setGroup(gr);
                    queueMove(0, m);
                }
            }

            m.setAction(ActionType::MOVE);
            m.setX(dest.x - vehicles_[vId].pos.x);
            m.setY(dest.y - vehicles_[vId].pos.y);
            queueMove(0, m);

            state = State::Move;
        }
        break;

        case State::Move: {
            state = State::Idle;
            return nukeStriker();
        }

        case State::Nuke: {
            static bool waitingMoveQueue = false;
            if (waitingMoveQueue)
                break;
            if (ctx_.me->getNextNuclearStrikeTickIndex() > 0) {
                if (ctx_.me->getNextNuclearStrikeVehicleId() == vId) {
                    break;
                } else {
                    state = State::Idle;
                    return false;
                }
            }
            if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
                state = State::Idle;
                return false;
            }

            const double visionCoeff = getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)vehicles_[vId].pos.x/32][(int)vehicles_[vId].pos.y/32]);
            const double visionSq = (vehicles_[vId].v.getVisionRange()*visionCoeff)*(vehicles_[vId].v.getVisionRange()*visionCoeff);
            V2d t{0., 0.};
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.isMine || v.second.v.getSquaredDistanceTo(vehicles_[vId].pos.x, vehicles_[vId].pos.y) > visionSq)
                    continue;
                t += v.second.pos;
                ++cnt;
            }
            if (!cnt) {
                return false;
            }
            t /= cnt;

            const double nSq = ctx_.game->getTacticalNuclearStrikeRadius()*ctx_.game->getTacticalNuclearStrikeRadius();
            for (const auto &v: vehicles_) {
                if (!v.second.isMine)
                    continue;
                if (v.second.v.getSquaredDistanceTo(t.x, t.y) < nSq) {
                    return false;
                }
            }

            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(t.x);
            m.setY(t.y);
            m.setVehicleId(vId);
            queueMove(0, m, [&waitingMoveQueue](){ waitingMoveQueue = false; });
            waitingMoveQueue = true;
        }
        break;

        case State::End:
            return false;
    }

    return true;
}

bool MyStrategy::onlyGroupSelected(int g) const {
    for (const auto &vext: vehicles_) {
        if (!vext.second.isMine)
            continue;
        const auto &groups = vext.second.v.getGroups();
        if (std::find(groups.begin(), groups.end(), g) == groups.end()) {
            if (vext.second.v.isSelected()) {
                return false;
            }
        } else if (!vext.second.v.isSelected()) {
            return false;
        }
    }
    return true;
}

void MyStrategy::manageFacilities() {
    enum class State {
        Idle,
        Producing,
        FinishedProduction,
        ControlCenter,
        Unknown
    };
    static std::unordered_set<long long> moveQueued;

//    bool switchedToHeli = false;

    for (const auto &f: ctx_.world->getFacilities()) {
        if (f.getOwnerPlayerId() != ctx_.me->getId()) {
            moveQueued.erase(f.getId());
//            scaleQueued.erase(f.getId());
            continue;
        }
        if (moveQueued.count(f.getId())) {
            continue;
        }
        auto state = State::Unknown;
        if (f.getType() == FacilityType::CONTROL_CENTER) {
            state = State::ControlCenter;
        } else if (f.getVehicleType() == VehicleType::_UNKNOWN_) {
            state = State::Idle;
        } else if (f.getProductionProgress()) {
            state = State::Producing;
        } else {
            state = State::FinishedProduction;
        }

        switch (state) {
            case State::Idle: {
/*
                constexpr double offset = 16.;
                bool gotMainGroup = false;
                for (const auto &vext: vehicles_) {
                    if (!vext.second.isMine ||
                        vext.second.pos.x < f.getLeft() - offset ||
                        vext.second.pos.x > f.getLeft() + ctx_.game->getFacilityWidth() + offset ||
                        vext.second.pos.y < f.getTop() - offset ||
                        vext.second.pos.y > f.getTop() + ctx_.game->getFacilityHeight() + offset) {
                        continue;
                    }

                    if (std::find(vext.second.v.getGroups().begin(),
                                  vext.second.v.getGroups().end(),
                                  MAIN_GROUP) != vext.second.v.getGroups().end()) {
                        gotMainGroup = true;
                        break;
                    }
                }
                if (gotMainGroup)
                    break;
*/
                Move m;
                m.setAction(ActionType::SETUP_VEHICLE_PRODUCTION);
                m.setFacilityId(f.getId());
                m.setVehicleType(recommendVehicleType());
                queueMove(0, m, [&moveQueued, fid = f.getId()]() { moveQueued.erase(fid); });
                moveQueued.insert(f.getId());
            }
            break;

            case State::ControlCenter:
            case State::Producing:
            break;

            case State::FinishedProduction: {
                int cntFigters = 0;
                for (const auto &vext: vehicles_) {
                    if (vext.second.isMine && vext.second.v.getType() == VehicleType::FIGHTER) {
                        ++cntFigters;
                    }
                }

                if (!cntFigters) {
                    if (f.getVehicleType() != VehicleType::FIGHTER) {
                        Move m;
                        m.setAction(ActionType::SETUP_VEHICLE_PRODUCTION);
                        m.setVehicleType(VehicleType::FIGHTER);
                        m.setFacilityId(f.getId());
                        queueMove(0, m, [&moveQueued, fid = f.getId()]() { moveQueued.erase(fid); });
                        moveQueued.insert(f.getId());
                    }
                } else if (f.getVehicleType() == VehicleType::FIGHTER) {
                    Move m;
                    m.setAction(ActionType::SETUP_VEHICLE_PRODUCTION);
                    m.setVehicleType(recommendVehicleType());
                    m.setFacilityId(f.getId());
                    queueMove(0, m, [&moveQueued, fid = f.getId()]() { moveQueued.erase(fid); });
                    moveQueued.insert(f.getId());
                }/* else if (!switchedToHeli) {
                    bool canSwitchToHeli = false;
                    for (const auto &of: ctx_.world->getFacilities()) {
                        if (of.getId() == f.getId() ||
                            of.getType() != FacilityType::VEHICLE_FACTORY ||
                            of.getOwnerPlayerId() != ctx_.me->getId()) {
                            continue;
                        }
                        if (of.getVehicleType() == VehicleType::IFV) {
                            canSwitchToHeli = true;
                        }
                    }
                    if (canSwitchToHeli) {
                        Move m;
                        m.setAction(ActionType::SETUP_VEHICLE_PRODUCTION);
                        m.setVehicleType(VehicleType::HELICOPTER);
                        m.setFacilityId(f.getId());
                        queueMove(0, m, [&moveQueued, fid = f.getId()]() { moveQueued.erase(fid); });
                        moveQueued.insert(f.getId());
                        switchedToHeli = true;
                    }
                }*/
            }
            break;

            case State::Unknown:
            break;
        }
    }
}

void MyStrategy::manageClusters() {
    if (!clist_) {
        clist_ = Clusterize::clusterize2(ctx_, 10.);
    }

    for (const auto &c: clist_->myClusters) {
        bool gotSomeGroup = false;
        bool isMoving = false;
        for (auto vid: c.set) {
            if (vehicles_[vid].hasMoved()) {
                isMoving = true;
                break;
            }
            for (auto gr: vehicles_[vid].v.getGroups()) {
                switch (gr) {
                    case MAIN_GROUP:
                    case NUKE_GROUP:
                        gotSomeGroup = true;
                        break;
                }
            }
            if (gotSomeGroup) {
                break;
            }
        }
        if (gotSomeGroup || isMoving) {
            continue;
        }

        bool onFacility = false;
        constexpr double offset = 10.;
        for (const auto &f: ctx_.world->getFacilities()) {
            if (onFacility) {
                break;
            }
            if (f.getOwnerPlayerId() != ctx_.me->getId() ||
                f.getType() != FacilityType::VEHICLE_FACTORY) {
                continue;
            }
            const double fxmin = f.getLeft() - offset,
                    fxmax = f.getLeft()+ctx_.game->getFacilityWidth()+offset,
                    fymin = f.getTop() - offset,
                    fymax = f.getTop()+ctx_.game->getFacilityHeight()+offset;
            for (auto vid: c.set) {
                if (vehicles_[vid].pos.x > fxmin &&
                    vehicles_[vid].pos.x < fxmax &&
                    vehicles_[vid].pos.y > fymin &&
                    vehicles_[vid].pos.y < fymax) {
                    onFacility = true;
                    break;
                }
            }
        }

        if (c.set.size() < 33 && onFacility) {
            continue;
        }

        double xmin = ctx_.world->getWidth(),
                xmax = 0.,
                ymin = ctx_.world->getHeight(),
                ymax = 0.;
        for (auto vid: c.set) {
            xmin = std::min(xmin, vehicles_[vid].pos.x);
            xmax = std::max(xmax, vehicles_[vid].pos.x);
            ymin = std::min(ymin, vehicles_[vid].pos.y);
            ymax = std::max(ymax, vehicles_[vid].pos.y);
        }
        xmin -= 5.;
        xmax += 5.;
        ymin -= 5.;
        ymax += 5.;
        V2d center = {(xmin+xmax)/2., (ymin+ymax)/2.};

        std::array<bool, (int)VehicleType::_COUNT_> types;
        types.fill(false);
        for (auto vid: c.set) {
            types[(int)vehicles_[vid].v.getType()] = true;
        }

        // check for compression
        constexpr double min_ratio = 3.;
        const double ratio = (xmax - xmin)/(ymax - ymin);
        if (!onFacility &&
            c.set.size() > 10 &&
            (ratio > min_ratio || ratio < 1./min_ratio)) {
            Move m;

            bool firstSelect = true;
            for (unsigned i=0; i < types.size(); ++i) {
                if (!types[i]) {
                    continue;
                }
                if (firstSelect) {
                    firstSelect = false;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(xmin);
                    m.setTop(ymin);
                    m.setRight(xmax);
                    m.setBottom(ymax);
                } else {
                    m.setAction(ActionType::ADD_TO_SELECTION);
                }
                m.setVehicleType((VehicleType)i);
                queueMove(0, m);
            }

            m.setAction(ActionType::ROTATE);
            m.setX(center.x);
            m.setY(center.y);
            m.setAngle(PI/2.);
            queueMove(0, m);

            firstSelect = true;
            for (unsigned i=0; i < types.size(); ++i) {
                if (!types[i]) {
                    continue;
                }
                if (firstSelect) {
                    firstSelect = false;
                    m.setAction(ActionType::CLEAR_AND_SELECT);
                    m.setLeft(std::min(xmin, center.x - (ymax - center.y)));
                    m.setTop(std::min(ymin, center.y - (center.x - xmin)));
                    m.setRight(std::max(xmax, center.x + (center.y - ymin)));
                    m.setBottom(std::max(ymax, center.y + (xmax - center.x)));
                } else {
                    m.setAction(ActionType::ADD_TO_SELECTION);
                }
                m.setVehicleType((VehicleType)i);
                queueMove(30, m);
            }

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(30, m);
        } else if (c.isAir) {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(xmin);
            m.setTop(ymin);
            m.setRight(xmax);
            m.setBottom(ymax);
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(MAIN_GROUP);
            queueMove(0, m);

            m.setGroup(AIR_GROUP);
            queueMove(0, m);

            //m.setGroup(ANTINUKE_GROUP);
            //queueMove(0, m);
        } else {
            V2d t = target(center, true, false);

            constexpr double etaSq = 4.;
            if ((t - center).getNormSq() > etaSq) {
                Move m;

                bool firstSelect = true;
                for (unsigned i=0; i < types.size(); ++i) {
                    if (!types[i]) {
                        continue;
                    }
                    if (firstSelect) {
                        firstSelect = false;
                        m.setAction(ActionType::CLEAR_AND_SELECT);
                        m.setLeft(xmin);
                        m.setTop(ymin);
                        m.setRight(xmax);
                        m.setBottom(ymax);
                    } else {
                        m.setAction(ActionType::ADD_TO_SELECTION);
                    }
                    m.setVehicleType((VehicleType)i);
                    queueMove(0, m);
                }

                m.setAction(ActionType::MOVE);
                m.setX(t.x - center.x);
                m.setY(t.y - center.y);
                queueMove(0, m);
            }
        }
    }
}

VehicleType MyStrategy::recommendVehicleType() const {
    VehicleType res = VehicleType::TANK;
    int enAir = 0,
        enGround = 0,
        myIFV = 0,
        myTank = 0;
    for (const auto &vext: vehicles_) {
        if (vext.second.isMine) {
            switch (vext.second.v.getType()) {
                case VehicleType::IFV: ++myIFV; break;
                case VehicleType::TANK: ++myTank; break;
                default: ;
            }
        } else {
            if (vext.second.v.isAerial()) {
                ++enAir;
            } else {
                ++enGround;
            }
        }
    }

    if (enAir > enGround && myIFV < enAir*2) {
        res = VehicleType::IFV;
    } else {
        res = VehicleType::TANK;
    }

    return res;
}
