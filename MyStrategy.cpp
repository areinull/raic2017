#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace model;
using namespace std;

namespace {
    constexpr int LAND_GROUP = 1;
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

MyStrategy::MyStrategy() {
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
    }
}

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()] = vehicle;
        vehiclesUpdateTick_[vehicle.getId()] = world.getTickIndex();
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
            vehiclesUpdateTick_.erase(vehicleId);
        } else {
            vehicles_[vehicleId] = Vehicle(vehicles_[vehicleId], vehicleUpdate);
            vehiclesUpdateTick_[vehicleId] = world.getTickIndex();
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.begin()->first <= ctx_.world->getTickIndex()) {
        move = moveQueue_.begin()->second;
        moveQueue_.erase(moveQueue_.begin());
    }

    return true;
}

std::pair<double, double> MyStrategy::center() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double, double> MyStrategy::target() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double,double> MyStrategy::span() const {
    double x_min = ctx_.world->getWidth();
    double x_max = 0.;
    double y_min = ctx_.world->getHeight();
    double y_max = 0.;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x_min = std::min(x_min, v.second.getX());
            x_max = std::max(x_max, v.second.getX());
            y_min = std::min(y_min, v.second.getY());
            y_max = std::max(y_max, v.second.getY());
        }
    }
    return {x_max-x_min, y_max-y_min};
}

void MyStrategy::nuke(const std::pair<double, double> &c, std::pair<double, double> &nukePos, long long &strikeUnit) {
    if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
        return;
    }

    const auto t = target();

    auto eDir = std::make_pair(t.first - c.first, t.second - c.second);
    const auto eDirL = std::sqrt(eDir.first*eDir.first + eDir.second*eDir.second);
    eDir.first /= eDirL;
    eDir.second /= eDirL;

    const auto quarterSpan = 32.;
    const auto pointA = std::make_pair(c.first + eDir.first*quarterSpan, c.second + eDir.second*quarterSpan);

    // choose strike vehicle near A
    long long vehicleId = -1;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId())
            continue;

        const double visionCoeff = v.second.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]);
        const double score = v.second.getSquaredDistanceTo(pointA.first, pointA.second) - v.second.getVisionRange()*visionCoeff*v.second.getVisionRange()*visionCoeff;
        if (score < best_score) {
            best_score = score;
            vehicleId = v.first;
        }
    }
    const auto &v = vehicles_[vehicleId];

    // choose strike point
    const double strikeDistance = v.getVisionRange()*0.9;
    std::pair<double, double> pointB = t;
    if (v.getSquaredDistanceTo(t.first, t.second) > strikeDistance*strikeDistance) {
        auto tDir = std::make_pair(t.first - v.getX(), t.second - v.getY());
        const auto tDirL = std::sqrt(tDir.first*tDir.first + tDir.second*tDir.second);
        tDir.first /= tDirL;
        tDir.second /= tDirL;
        pointB.first = v.getX() + tDir.first*strikeDistance;
        pointB.second = v.getY() + tDir.second*strikeDistance;
    }

    // estimate damage in strike zone
    double balanceDamage = 0.;
    for (const auto &v: vehicles_) {
        const auto d = v.second.getDistanceTo(pointB.first, pointB.second);
        if ( d > ctx_.game->getTacticalNuclearStrikeRadius())
            continue;
        const double dmg = ctx_.game->getMaxTacticalNuclearStrikeDamage() * (1 - d/ctx_.game->getTacticalNuclearStrikeRadius());
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            balanceDamage -= dmg;
        } else {
            balanceDamage += dmg;
        }
    }

    // strike!
    if (balanceDamage > 10.*ctx_.game->getMaxTacticalNuclearStrikeDamage()) {
        nukePos = pointB;
        strikeUnit = vehicleId;
    }
}


void MyStrategy::move() {
    startupGroundFormation() || mainGround();
    mainFighter();
    mainHeli();
    return;
/*
    if (world.getTickIndex() == 0) {
        congregate();
    }
    if (world.getTickIndex() == 240) {
        congregate();
    }
    if (world.getTickIndex() == 480) {
        congregate();
    }

    if (world.getTickIndex() >= 720) {
#ifdef MYDEBUG
        std::cout << "tick: " << world.getTickIndex() << " arState: " << antiReconState_;
#endif

        if (antiReconState_) {
            if (--antiReconDelay_ > 0) {
#ifdef MYDEBUG
                std::cout << " antiReconDelay: " << antiReconDelay_ << std::endl;
#endif
            } else {
                detectRecon(false);
                attackRecon();
            }
#ifdef MYDEBUG
            std::cout << std::endl;
#endif
            return;
        }

        nuke();

        if (detectRecon(true))
            return;

        if (needCongregate_) {
            needCongregate_ = false;
            congregate();
        }

        if (world.getTickIndex() % 120 == 0) {
            if (distToEnemy() > 20.) {
                constexpr double dd_max = 100. * 100.;
                const auto c = center();
                const auto t = target();
                double dx = t.first - c.first;
                double dy = t.second - c.second;
                const double dd = dx * dx + dy * dy;
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    dx *= k;
                    dy *= k;
                }

                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                queueMove(0, m);
                m.setAction(ActionType::MOVE);
                m.setX(dx);
                m.setY(dy);
                m.setMaxSpeed(game.getTankSpeed());
                queueMove(0, m);
#ifdef MYDEBUG
                std::cout << " move " << dx << ' ' << dy;
#endif
            } else {
                congregate();
#ifdef MYDEBUG
                std::cout << " congregate";
#endif
            }
        }
#ifdef MYDEBUG
        std::cout << std::endl;
#endif
    }
    */
}

MyStrategy::VehicleSet MyStrategy::detectRecon(bool isAir) {
    VehicleSet res;
    constexpr int maxRecon = 20;
    std::pair<double, double> c;
    {
        double x = 0.;
        double y = 0.;
        int cnt = 0;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                x += v.second.getX();
                y += v.second.getY();
                ++cnt;
            }
        }
        if (cnt) {
            x /= cnt;
            y /= cnt;
        }
        c.first = x;
        c.second = y;
    }

    const double leftBorder = c.first - 50. - ctx_.game->getFighterVisionRange()*2.;
    const double rightBorder = c.first + 50. + ctx_.game->getFighterVisionRange()*2.;
    const double topBorder = c.second - 50. - ctx_.game->getFighterVisionRange()*2.;
    const double bottomBorder = c.second + 50/2. + ctx_.game->getFighterVisionRange()*2.;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() || (isAir && !v.second.isAerial()) || (!isAir && v.second.isAerial()))
            continue;
        if (v.second.getX() > leftBorder && v.second.getX() < rightBorder && v.second.getY() > topBorder && v.second.getY() < bottomBorder) {
            res.insert(v.first);
        }
    }

    if (res.size() > maxRecon)
        res.clear();

    return res;
}

bool MyStrategy::startupGroundFormation() {
    enum class State {
        InitialScale,
        InitialReposition,
        Rotate,
        Spread,
        Shift,
        Mix,
        ScaleBeforeRotate,
        FinalRotate,
        Stack,
        FinalScale,
        End
    };
    static State state{State::InitialScale};

    static std::array<VehicleType, 3> grNum{VehicleType::TANK, VehicleType::IFV, VehicleType::ARRV};
    static std::map<VehicleType, int> type2idx {
            {VehicleType::TANK, 0},
            {VehicleType::IFV,  1},
            {VehicleType::ARRV, 2}
    };
    static std::array<std::pair<double, double>, 3> grPos;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial() && ctx_.world->getTickIndex() - vehiclesUpdateTick_[v.first] < 10) {
            // юниты ещё перемещаются
            return true;
        }
    }

    switch (state) {
        case State::InitialScale: {
            grPos.fill(std::make_pair(0., 0.));

            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId())
                    continue;
                switch (v.second.getType()) {
                    case VehicleType::TANK:
                    case VehicleType::IFV:
                    case VehicleType::ARRV:
                        break;
                    default:
                        continue;
                }
                auto &pos = grPos[type2idx[v.second.getType()]];
                pos.first += v.second.getX() / 100.;
                pos.second += v.second.getY() / 100.;
            }

            {
                int firstGr = 0;
                double minY = grPos[0].second;
                for (int i: {1, 2}) {
                    if (grPos[i].second < minY) {
                        minY = grPos[i].second;
                        firstGr = i;
                    }
                }
                double minX = grPos[firstGr].first;
                for (int i: {(firstGr + 1) % 3, (firstGr + 2) % 3}) {
                    if (grPos[i].second <= minY && grPos[i].first < minX) {
                        minX = grPos[i].first;
                        firstGr = i;
                    }
                }
                std::swap(grNum[0], grNum[firstGr]);
                std::swap(grPos[0], grPos[firstGr]);

                if (grPos[1].second > grPos[2].second || grPos[1].first < grPos[2].first) {
                    std::swap(grNum[2], grNum[1]);
                    std::swap(grPos[2], grPos[1]);
                }

                for (int i = 0; i < 3; ++i) {
                    type2idx[grNum[i]] = i;
                }
            }

            for (auto vt: grNum) {
                double left = ctx_.world->getWidth();
                double right = 0.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != vt)
                        continue;
                    left = std::min(left, v.second.getX());
                    right = std::max(right, v.second.getX());
                }
                const double spacing = (right - left) / 9.;

                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(vt);
                queueMove(0, m);

                m.setAction(ActionType::SCALE);
                m.setX(grPos[type2idx[vt]].first);
                m.setY(grPos[type2idx[vt]].second);
                m.setFactor(5. / spacing);
                queueMove(0, m);
            }
            state = State::InitialReposition;
        }
        break;

        case State::InitialReposition: {
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[0]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(325. - grPos[0].first);
                m.setY(100. - grPos[0].second);
                queueMove(0, m);
                grPos[0] = {325., 100.};
            }
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[1]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(grPos[0].first - 150.*M_SQRT1_2 - grPos[1].first);
                m.setY(grPos[0].second + 150.*M_SQRT1_2 - grPos[1].second);
                queueMove(0, m);
                grPos[1] = {grPos[0].first - 150.*M_SQRT1_2, grPos[0].second + 150.*M_SQRT1_2};
            }
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[2]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(100. - grPos[2].first);
                m.setY(325. - grPos[2].second);
                queueMove(0, m);
                grPos[2] = {100., 325.};
            }
            state = State::Rotate;
        }
        break;

        case State::Rotate: {
            for (auto vt: grNum) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(vt);
                queueMove(0, m);

                m.setAction(ActionType::ROTATE);
                m.setX(grPos[type2idx[vt]].first);
                m.setY(grPos[type2idx[vt]].second);
                m.setAngle(M_PI_4);
                queueMove(0, m);
            }
            state = State::Spread;
        }
        break;

        case State::Spread: {
            for (auto vt: grNum) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(vt);
                queueMove(0, m);

                m.setAction(ActionType::SCALE);
                m.setX(grPos[type2idx[vt]].first);
                m.setY(grPos[type2idx[vt]].second);
                m.setFactor(3.);
                queueMove(0, m);
            }

            state = State::Shift;
        }
        break;

        case State::Shift: {
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[0]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(5. * M_SQRT1_2);
                m.setY(5. * M_SQRT1_2);
                queueMove(0, m);

                grPos[0].first += 5. * M_SQRT1_2;
                grPos[0].second += 5. * M_SQRT1_2;
            }
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[2]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(-5. * M_SQRT1_2);
                m.setY(-5. * M_SQRT1_2);
                queueMove(0, m);

                grPos[2].first -= 5. * M_SQRT1_2;
                grPos[2].second -= 5. * M_SQRT1_2;
            }
            state = State::Mix;
        }
        break;

        case State::Mix: {
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[0]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(grPos[1].first + 5. * M_SQRT1_2 - grPos[0].first);
                m.setY(grPos[1].second + 5. * M_SQRT1_2 - grPos[0].second);
                queueMove(0, m);

                grPos[0] = {grPos[1].first + 5. * M_SQRT1_2, grPos[1].second + 5. * M_SQRT1_2};
            }
            {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                m.setVehicleType(grNum[2]);
                queueMove(0, m);

                m.setAction(ActionType::MOVE);
                m.setX(grPos[1].first - 5. * M_SQRT1_2 - grPos[2].first);
                m.setY(grPos[1].second - 5. * M_SQRT1_2 - grPos[2].second);
                queueMove(0, m);

                grPos[2] = {grPos[1].first - 5. * M_SQRT1_2, grPos[1].second - 5. * M_SQRT1_2};
            }
            state = State::ScaleBeforeRotate;
        }
        break;

        case State::ScaleBeforeRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(grNum[0]);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(grNum[1]);
            queueMove(0, m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(grNum[2]);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(grPos[1].first);
            m.setY(grPos[1].second);
            m.setFactor(1.1);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            state = State::FinalRotate;
        }
        break;

        case State::FinalRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(grPos[1].first);
            m.setY(grPos[1].second);
            m.setAngle(-M_PI_4);
            queueMove(0, m);

            state = State::Stack;
        }
        break;

        case State::Stack: {
            double yMin = ctx_.world->getHeight();
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId() || v.second.isAerial())
                    continue;
                yMin = std::min(yMin, v.second.getY());
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::DESELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(yMin + 5.);
            m.setGroup(0);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(0);
            m.setY(-512.);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::FinalScale;
        }
        break;

        case State::FinalScale: {
            auto pos = std::make_pair(0., 0.);
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != ctx_.me->getId() || v.second.isAerial())
                    continue;
                pos.first += v.second.getX()/300.;
                pos.second += v.second.getY()/300.;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(0.5);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::End;
        }
        break;

        case State::End:
            return false;
    }
    return true;
}

void MyStrategy::queueMove(int delay, const Move &m){
    moveQueue_.emplace(ctx_.world->getTickIndex() + delay, m);
}

bool MyStrategy::mainGround() {
    enum class State {
        Idle,
        PreRotate,
        Rotate,
        PostRotate,
        Move,
        NukeStrike,
        NukeStrikeWait,
        End
    };
    static State state{State::Idle};
    static pair<double, double> pos{0., 0.},
                                rot{0., 1.},
                                t{0., 0.},
                                d{0., 0.};
    constexpr double dd_max = 64.;
    static double ang = 0.;

    if (state == State::End)
        return false;

    std::pair<double, double> nukePos;
    long long strikeUnit = -1;
    nuke(pos, nukePos, strikeUnit);
    if (strikeUnit >= 0) {
        state = State::NukeStrike;
    }

    if (state != State::NukeStrike) {
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial() &&
                ctx_.world->getTickIndex() - vehiclesUpdateTick_[v.first] < 10) {
                // юниты ещё перемещаются
                return true;
            }
        }
    }

    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId() || v.second.isAerial())
            continue;
        pos.first += v.second.getX();
        pos.second += v.second.getY();
        ++cnt;
    }
    if (!cnt) {
        // наземных юнитов не осталось
        state = State::End;
        return false;
    }
    pos.first /= cnt;
    pos.second /= cnt;

    switch (state) {
        case State::Idle: {
            t = target();
            d.first = t.first - pos.first;
            d.second = t.second - pos.second;
            double dd = std::sqrt(d.first * d.first + d.second * d.second);
            ang = std::asin((rot.first * d.second - rot.second * d.first) / dd);
            if (dd > dd_max) {
                const double k = dd_max / dd;
                d.first *= k;
                d.second *= k;
                dd = dd_max;
            }
            if (std::abs(ang) > M_PI / 12.) {
                rot.first = d.first / dd;
                rot.second = d.second / dd;
                state = State::PreRotate;
            } else {
                state = State::Move;
            }
            return mainGround();
        }

        case State::PreRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(1.1);
            queueMove(0, m);

            state = State::Rotate;
        }
        break;

        case State::Rotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle(ang);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::PostRotate;
        }
        break;

        case State::PostRotate: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setFactor(0.1);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::Move: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(d.first);
            m.setY(d.second);
            m.setMaxSpeed(slowestGroundSpeed_);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::NukeStrike: {
            Move m;
            m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
            m.setX(nukePos.first);
            m.setY(nukePos.second);
            m.setVehicleId(strikeUnit);
            queueMove(0, m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setGroup(LAND_GROUP);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(0);
            m.setY(0);
            queueMove(0, m);

            state = State::NukeStrikeWait;
        }
        break;

        case State::NukeStrikeWait: {
            if (ctx_.me->getNextNuclearStrikeTickIndex() < 0) {
                state = State::Idle;
                return mainGround();
            }
        }
        break;

        case State::End:
            return false;
    }
    return true;
}

bool MyStrategy::mainHeli() {
    enum class State {
        Idle,
        Converge,
        Move,
        AntiRecon,
        End
    };
    static auto state = State::Idle;
    static std::pair<double, double> pos{0., 0.};
    static VehicleSet enemyRecon;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::HELICOPTER &&
            ctx_.world->getTickIndex() - vehiclesUpdateTick_[v.first] < 10) {
            // юниты ещё перемещаются
            return true;
        }
    }

    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != VehicleType::HELICOPTER)
            continue;
        pos.first += v.second.getX();
        pos.second += v.second.getY();
        ++cnt;
    }
    if (!cnt) {
        state = State::End;
        return false;
    }
    pos.first /= cnt;
    pos.second /= cnt;

    switch (state) {
        case State::Idle: {
            enemyRecon = detectRecon(false);
            if (!enemyRecon.empty()) {
                state = State::AntiRecon;
            } else {
                double x_min = ctx_.world->getWidth(),
                        x_max = 0.,
                        y_min = ctx_.world->getHeight(),
                        y_max = 0.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::HELICOPTER) {
                        x_min = std::min(x_min, v.second.getX());
                        x_max = std::max(x_max, v.second.getX());
                        y_min = std::min(y_min, v.second.getY());
                        y_max = std::max(y_max, v.second.getY());
                    }
                }
                const double x_span = x_max - x_min,
                        y_span = y_max - y_min;
                if (x_span > 100. || y_span > 100.) {
                    state = State::Converge;
                } else {
                    state = State::Move;
                }
            }
            return mainHeli();
        }

        case State::Converge: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle((rand()%2 == 0? 1.: -1.) * M_PI_4);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(20, m);

            state = State::Idle;
        }
        break;

        case State::Move: {
            std::pair<double, double> ally{0., 0.},
                                      foe{0., 0.},
                                      dest{0., 0.};
            int cnt_ally = 0, cnt_foe = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    ally.first += v.second.getX();
                    ally.second += v.second.getY();
                    ++cnt_ally;
                } else if (v.second.getPlayerId() != ctx_.me->getId() && !v.second.isAerial()) {
                    foe.first += v.second.getX();
                    foe.second += v.second.getY();
                    ++cnt_foe;
                }
            }
            if (cnt_foe) {
                foe.first /= cnt_foe;
                foe.second /= cnt_foe;
            }
            if (cnt_ally) {
                ally.first /= cnt_ally;
                ally.second /= cnt_ally;
            } else {
                // наземных юнитов не осталось, движемся к врагу
                ally = foe;
            }

            const double toFoeX = foe.first - ally.first;
            const double toFoeY = foe.second - ally.second;
            const double toFoeD = std::sqrt(toFoeX*toFoeX + toFoeY*toFoeY);

            dest.first = clampX(ally.first + toFoeX/toFoeD*32. + (rand()%200)-100);
            dest.second = clampY(ally.second + toFoeY/toFoeD*32. + (rand()%200)-100);


            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dest.first - pos.first);
            m.setY(dest.second - pos.second);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::AntiRecon: {
            long long closest = -1;
            double minSqDist = std::numeric_limits<double>::max();
            for (auto vid: enemyRecon) {
                const double sqDist = vehicles_[vid].getSquaredDistanceTo(pos.first, pos.second);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    closest = vid;
                }
            }

            const double dist = std::sqrt(minSqDist);
            double dx = vehicles_[closest].getX() - pos.first;
            double dy = vehicles_[closest].getY() - pos.second;
            if (dist > 200.) {
                const double k = 200./dist;
                dx *= k;
                dy *= k;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dx);
            m.setY(dy);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::End:
            return false;
    }

    return true;
}

bool MyStrategy::mainFighter() {
    enum class State {
        Idle,
        Converge,
        Move,
        AntiRecon,
        End
    };
    static auto state = State::Idle;
    static std::pair<double, double> pos{0., 0.};
    static VehicleSet enemyRecon;

    if (state == State::End)
        return false;

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::FIGHTER &&
            ctx_.world->getTickIndex() - vehiclesUpdateTick_[v.first] < 10) {
            // юниты ещё перемещаются
            return true;
        }
    }

    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != VehicleType::FIGHTER)
            continue;
        pos.first += v.second.getX();
        pos.second += v.second.getY();
        ++cnt;
    }
    if (!cnt) {
        state = State::End;
        return false;
    }
    pos.first /= cnt;
    pos.second /= cnt;

    switch (state) {
        case State::Idle: {
            enemyRecon = detectRecon(true);
            if (!enemyRecon.empty()) {
                state = State::AntiRecon;
            } else {
                double x_min = ctx_.world->getWidth(),
                        x_max = 0.,
                        y_min = ctx_.world->getHeight(),
                        y_max = 0.;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == ctx_.me->getId() && v.second.getType() == VehicleType::FIGHTER) {
                        x_min = std::min(x_min, v.second.getX());
                        x_max = std::max(x_max, v.second.getX());
                        y_min = std::min(y_min, v.second.getY());
                        y_max = std::max(y_max, v.second.getY());
                    }
                }
                const double x_span = x_max - x_min,
                        y_span = y_max - y_min;
                if (x_span > 100. || y_span > 100.) {
                    state = State::Converge;
                } else {
                    state = State::Move;
                }
            }
            return mainHeli();
        }

        case State::Converge: {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::ROTATE);
            m.setX(pos.first);
            m.setY(pos.second);
            m.setAngle((rand()%2 == 0? 1.: -1.) * M_PI_4);
            queueMove(0, m);

            m.setAction(ActionType::SCALE);
            m.setFactor(0.1);
            queueMove(20, m);

            state = State::Idle;
        }
            break;

        case State::Move: {
            std::pair<double, double> ally{0., 0.},
                    foe{0., 0.},
                    dest{0., 0.};
            int cnt_ally = 0, cnt_foe = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    ally.first += v.second.getX();
                    ally.second += v.second.getY();
                    ++cnt_ally;
                } else if (v.second.getPlayerId() != ctx_.me->getId() && v.second.isAerial()) {
                    foe.first += v.second.getX();
                    foe.second += v.second.getY();
                    ++cnt_foe;
                }
            }
            if (cnt_foe) {
                foe.first /= cnt_foe;
                foe.second /= cnt_foe;
            }
            if (cnt_ally) {
                ally.first /= cnt_ally;
                ally.second /= cnt_ally;
            } else {
                // наземных юнитов не осталось, движемся к врагу
                ally = foe;
            }

            const double toFoeX = foe.first - ally.first;
            const double toFoeY = foe.second - ally.second;
            const double toFoeD = std::sqrt(toFoeX*toFoeX + toFoeY*toFoeY);

            dest.first = clampX(ally.first + toFoeX/toFoeD*32. + (rand()%200)-100);
            dest.second = clampY(ally.second + toFoeY/toFoeD*32. + (rand()%200)-100);

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dest.first - pos.first);
            m.setY(dest.second - pos.second);
            queueMove(0, m);

            state = State::Idle;
        }
        break;

        case State::AntiRecon: {
            long long closest = -1;
            double minSqDist = std::numeric_limits<double>::max();
            for (auto vid: enemyRecon) {
                const double sqDist = vehicles_[vid].getSquaredDistanceTo(pos.first, pos.second);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    closest = vid;
                }
            }

            const double dist = std::sqrt(minSqDist);
            double dx = vehicles_[closest].getX() - pos.first;
            double dy = vehicles_[closest].getY() - pos.second;
            if (dist > 200.) {
                const double k = 200./dist;
                dx *= k;
                dy *= k;
            }

            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom(ctx_.world->getHeight());
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);

            m.setAction(ActionType::MOVE);
            m.setX(dx);
            m.setY(dy);
            queueMove(0, m);

            state = State::Idle;
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
