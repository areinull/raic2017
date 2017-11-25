#include <cstring>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "Context.h"
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

void MoveField::addUnit(double x, double y) {
    const int xi = x/gridStride_;
    const int yi = y/gridStride_;
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

void MoveField::addObstacle(double x, double y) {
    const int xi = x/gridStride_;
    const int yi = y/gridStride_;
    for (int x = clamp(xi-1), x_end = clamp(xi+1)+1; x < x_end; ++x) {
        for (int y = clamp(yi-1), y_end = clamp(yi+1)+1; y < y_end; ++y) {
            field_[x][y] = 100;
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

std::vector<std::pair<double, double>> MoveField::pathToNeg(double x, double y) const {
    const auto p = pathToNeg(collapse(x/gridStride_, y/gridStride_));
    std::vector<std::pair<double, double>> res;
    res.reserve(p.size());
    std::transform(p.rbegin(), p.rend(), std::back_inserter(res), [](const int i) -> std::pair<double, double> {
        const auto e = expand(i);
        return {e.first*gridStride_+gridStride_*0.5, e.second*gridStride_+gridStride_*0.5};
    });
    return res;
}

std::vector<int> MoveField::pathToNeg(int s) const {
    const auto s_exp = expand(s);
    std::queue<int> q;
    std::unordered_set<int> vis;
    std::array<int, gridMax_*gridMax_> parent;
    parent.fill(-1);

    int dest = -1;
    q.push(s);
    parent[s] = s;
    while (!q.empty()) {
        const auto cur = q.front();
        const auto cur_exp = expand(cur);
        q.pop();
        if (vis.count(cur)) {
            continue;
        }
        vis.insert(cur);

        if ((field_[s_exp.first][s_exp.second] > 0 && field_[cur_exp.first][cur_exp.second] <= 0) ||
            field_[cur_exp.first][cur_exp.second] < 0) {
            dest = cur;
            break;
        }

        for (const auto &dif_exp: {
                std::pair<int, int>{-1, 0},
                std::pair<int, int>{1, 0},
                std::pair<int, int>{0, -1},
                std::pair<int, int>{0, 1}
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
            q.push(n);
            parent[n] = cur;
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
