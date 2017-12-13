#include <cstring>
#include <iomanip>
#include <algorithm>
#include <map>
#include <unordered_set>
#include <iostream>
#include "Context.h"
#include "model/World.h"
#include "model/Game.h"
#include "MoveField.h"

MoveField::MoveField() {
    clearField();
}

void MoveField::clearField() {
    memset(field_, 0, gridMax_*gridMax_*sizeof(int));
}

int MoveField::clamp(int v) {
    return std::max(0, std::min(gridMax_-1, v));
}

void MoveField::addEnemyUnit(const V2d &p) {
    const int xi = p.x/gridStride_;
    const int yi = p.y/gridStride_;
    for (int x = clamp(xi-nukeRange_), x_end = clamp(xi+nukeRange_)+1; x < x_end; ++x) {
        for (int y = clamp(yi-nukeRange_), y_end = clamp(yi+nukeRange_)+1; y < y_end; ++y) {
            if (std::abs(xi-x) <= fireRange_ && std::abs(yi-y) <= fireRange_) {
                field_[x][y] = 100;
            } else if (field_[x][y] <= 0) {
                field_[x][y] -= 1;
            }
        }
    }
}

void MoveField::addFriendUnit(const V2d &p) {
    const int xi = p.x/gridStride_;
    const int yi = p.y/gridStride_;
    for (int x = clamp(xi-1), x_end = clamp(xi+1)+1; x < x_end; ++x) {
        for (int y = clamp(yi-1), y_end = clamp(yi+1)+1; y < y_end; ++y) {
            field_[x][y] = 100;
        }
    }
}

void MoveField::addPoint(const V2d &p, int v) {
    const int xi = p.x/gridStride_;
    const int yi = p.y/gridStride_;
    field_[xi][yi] += v;
}

void MoveField::addNuke(const V2d &p) {
    const int xi = p.x/gridStride_;
    const int yi = p.y/gridStride_;
    for (int x = clamp(xi-nukeRange_), x_end = clamp(xi+nukeRange_); x <= x_end; ++x) {
        for (int y = clamp(yi-nukeRange_), y_end = clamp(yi+nukeRange_); y <= y_end; ++y) {
            field_[x][y] = 100;
        }
    }
}

void MoveField::addWeather(const Context &ctx) {
    for (int x=0; x < gridMax_; ++x) {
        for (int y=0; y < gridMax_; ++y) {
            if (field_[x][y] < 0) {
                const int wx = (int) (x * gridStride_) / 32;
                const int wy = (int) (y * gridStride_) / 32;
                double k;
                switch (ctx.world->getWeatherByCellXY()[wx][wy]) {
                    case model::WeatherType::CLOUD:
                        k = ctx.game->getClearWeatherVisionFactor();
                        break;
                    case model::WeatherType::RAIN:
                        k = ctx.game->getRainWeatherVisionFactor();
                        break;
                    default:
                        k = ctx.game->getClearWeatherVisionFactor();
                }
                field_[x][y] = field_[x][y] * k;
            }
        }
    }
}

std::ostream& operator<<(std::ostream &s, const MoveField &f) {
    for (int y = 0; y < MoveField::gridMax_; ++y) {
        for (int x = 0; x < MoveField::gridMax_; ++x) {
            s << std::setw(4) << f.field_[x][y];
        }
        s << '\n';
    }
    return s;
}

int MoveField::collapse(int x, int y) {
    return x + y*gridMax_;
}

std::pair<int, int> MoveField::expand(int c) {
    return { c % gridMax_, c / gridMax_ };
}

int MoveField::value(const V2d &p) const {
    return field_[(int)(p.x/gridStride_)][(int)(p.y/gridStride_)];
}

bool MoveField::segmentClear(const V2d &a, const V2d &b) const {
    const auto u = (b - a).unit();
    const int n = (b - a).getNorm()/gridStride_;
    for (int i = 1; i <= n; ++i) {
        const auto c = a + u*gridStride_*i;
        if (value(c) > 0) {
            return false;
        }
    }
    return true;
}

std::vector<V2d> MoveField::pathToNeg(const V2d &s, bool global) const {
    const auto p = pathToNeg(collapse(s.x/gridStride_, s.y/gridStride_), global);
    std::vector<V2d> res;
    res.reserve(p.size());
    std::transform(p.rbegin(), p.rend(), std::back_inserter(res), [](const int i) -> V2d {
        const auto e = expand(i);
        return {e.first*gridStride_+gridStride_*0.5, e.second*gridStride_+gridStride_*0.5};
    });
    return res;
}

std::vector<int> MoveField::pathToNeg(int s, bool global) const {
    const auto s_exp = expand(s);
    std::multimap<int, std::pair<int, int>> fringe;
    std::unordered_set<int> vis;
    std::array<int, gridMax_*gridMax_> parent;
    parent.fill(-1);

    int dest = -1;
    fringe.emplace(0, std::make_pair(s, s));
    while (!fringe.empty()) {
        int cur_cost, cur, prev;
        std::pair<int, int> cur_edge;
        std::tie(cur_cost, cur_edge) = *fringe.begin();
        std::tie(prev, cur) = cur_edge;
        const auto cur_exp = expand(cur);
        fringe.erase(fringe.begin());
        if (vis.count(cur)) {
            continue;
        }
        vis.insert(cur);
        parent[cur] = prev;

        if ((field_[s_exp.first][s_exp.second] > 0 && field_[cur_exp.first][cur_exp.second] <= 0) ||
            (!global && field_[cur_exp.first][cur_exp.second] < 0) ||
            (global && field_[cur_exp.first][cur_exp.second] < field_[s_exp.first][s_exp.second])) {
            dest = cur;
            break;
        }
        if (global && cur_cost > 32) {
            break;
        }

        for (const auto &dif_exp: {
                std::pair<int, int>{-1, 0},
                std::pair<int, int>{1, 0},
                std::pair<int, int>{0, -1},
                std::pair<int, int>{0, 1},
                std::pair<int, int>{-1, -1},
                std::pair<int, int>{1, 1},
                std::pair<int, int>{1, -1},
                std::pair<int, int>{-1, 1}
        }) {
            const std::pair<int, int> n_exp{cur_exp.first+dif_exp.first, cur_exp.second+dif_exp.second};
            if (n_exp.first < 0 ||
                n_exp.first >= gridMax_ ||
                n_exp.second < 0 ||
                n_exp.second >= gridMax_ ||
                (field_[cur_exp.first][cur_exp.second] <= 0 && field_[n_exp.first][n_exp.second] > 0)) {
                continue;
            }
            const int n = collapse(n_exp.first, n_exp.second);
            if (vis.count(n)) {
                continue;
            }
            const int trans_cost = std::abs(dif_exp.first) + std::abs(dif_exp.second);
            fringe.emplace(cur_cost + trans_cost, std::make_pair(cur, n));
        }
    }

    if (dest < 0) {
        return {};
    }

    std::vector<int> res;
    for (int cur = dest; cur != s; cur = parent[cur]) {
        res.push_back(cur);
    }
    return res;
}
