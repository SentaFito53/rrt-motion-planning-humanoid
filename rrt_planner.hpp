#pragma once
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>

// ============================================================
//  RRT Planner sederhana untuk lapangan 2D (satuan: cm)
//  Obstacle direpresentasikan sebagai lingkaran (x, y, radius)
// ============================================================

struct Point2D {
    double x, y;
};

struct Obstacle {
    double x, y;
    double radius;   // radius aman = radius fisik robot lawan + margin robot sendiri
};

class RRTPlanner {
public:
    RRTPlanner(double mapMinX, double mapMaxX, double mapMinY, double mapMaxY,
               double stepSize = 20.0, int maxIterations = 3000, double goalTolerance = 15.0,
               double goalBias = 0.10)
        : minX(mapMinX), maxX(mapMaxX), minY(mapMinY), maxY(mapMaxY),
          step(stepSize), maxIter(maxIterations), goalTol(goalTolerance),
          goalBiasProb(goalBias), rng(std::random_device{}()) {}

    // Mengembalikan path dari start ke goal (list titik dunia nyata).
    // Path kosong berarti gagal menemukan jalur dalam batas iterasi.
    std::vector<Point2D> plan(Point2D start, Point2D goal,
                               const std::vector<Obstacle>& obstacles)
    {
        nodes.clear();
        nodes.push_back({start, -1});

        // Kalau start sendiri sudah nabrak/goal tak valid, tak usah lanjut
        if (pointInObstacle(start, obstacles) || pointInObstacle(goal, obstacles)) {
            return {};
        }

        std::uniform_real_distribution<double> distX(minX, maxX);
        std::uniform_real_distribution<double> distY(minY, maxY);
        std::uniform_real_distribution<double> distBias(0.0, 1.0);

        for (int i = 0; i < maxIter; i++) {
            Point2D rnd = (distBias(rng) < goalBiasProb)
                              ? goal
                              : Point2D{distX(rng), distY(rng)};

            int nearIdx = nearest(rnd);
            Point2D newPt = steer(nodes[nearIdx].pt, rnd);

            if (collides(nodes[nearIdx].pt, newPt, obstacles)) continue;

            nodes.push_back({newPt, nearIdx});

            if (dist(newPt, goal) <= goalTol && !collides(newPt, goal, obstacles)) {
                nodes.push_back({goal, (int)nodes.size() - 1});
                return extractPath((int)nodes.size() - 1);
            }
        }
        return {}; // gagal: tak ketemu jalur dalam maxIter percobaan
    }

private:
    struct RRTNode { Point2D pt; int parent; };
    std::vector<RRTNode> nodes;
    double minX, maxX, minY, maxY, step, goalTol, goalBiasProb;
    int maxIter;
    std::mt19937 rng;

    static double dist(const Point2D& a, const Point2D& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    int nearest(const Point2D& p) const {
        int best = 0;
        double bestDist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nodes.size(); i++) {
            double d = dist(nodes[i].pt, p);
            if (d < bestDist) { bestDist = d; best = (int)i; }
        }
        return best;
    }

    Point2D steer(const Point2D& from, const Point2D& to) const {
        double d = dist(from, to);
        if (d <= step) return to;
        double ratio = step / d;
        return {from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio};
    }

    bool pointInObstacle(const Point2D& p, const std::vector<Obstacle>& obstacles) const {
        for (const auto& o : obstacles) {
            if (dist(p, {o.x, o.y}) <= o.radius) return true;
        }
        return false;
    }

    // Cek apakah segmen from->to memotong salah satu lingkaran obstacle
    bool collides(const Point2D& from, const Point2D& to,
                  const std::vector<Obstacle>& obstacles) const
    {
        for (const auto& obs : obstacles) {
            if (segmentCircleIntersect(from, to, obs)) return true;
        }
        return false;
    }

    static bool segmentCircleIntersect(const Point2D& a, const Point2D& b, const Obstacle& obs) {
        double dx = b.x - a.x, dy = b.y - a.y;
        double fx = a.x - obs.x, fy = a.y - obs.y;

        double aCoef = dx * dx + dy * dy;
        if (aCoef < 1e-9) {
            // segmen berdegenerasi jadi titik
            return std::hypot(fx, fy) <= obs.radius;
        }
        double bCoef = 2 * (fx * dx + fy * dy);
        double cCoef = fx * fx + fy * fy - obs.radius * obs.radius;

        double disc = bCoef * bCoef - 4 * aCoef * cCoef;
        if (disc < 0) return false;

        disc = std::sqrt(disc);
        double t1 = (-bCoef - disc) / (2 * aCoef);
        double t2 = (-bCoef + disc) / (2 * aCoef);

        if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) return true;
        return cCoef < 0; // titik awal sudah di dalam lingkaran
    }

    std::vector<Point2D> extractPath(int goalIdx) const {
        std::vector<Point2D> path;
        int idx = goalIdx;
        while (idx != -1) {
            path.push_back(nodes[idx].pt);
            idx = nodes[idx].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// ------------------------------------------------------------
// Path smoothing opsional: hilangkan waypoint yang bisa "dilompati"
// langsung tanpa nabrak obstacle. Dipanggil terpisah setelah plan().
// ------------------------------------------------------------
inline std::vector<Point2D> smoothPath(const std::vector<Point2D>& path,
                                        const std::vector<Obstacle>& obstacles)
{
    if (path.size() < 3) return path;

    auto segmentClear = [&](const Point2D& a, const Point2D& b) {
        for (const auto& obs : obstacles) {
            double dx = b.x - a.x, dy = b.y - a.y;
            double fx = a.x - obs.x, fy = a.y - obs.y;
            double aCoef = dx * dx + dy * dy;
            if (aCoef < 1e-9) continue;
            double bCoef = 2 * (fx * dx + fy * dy);
            double cCoef = fx * fx + fy * fy - obs.radius * obs.radius;
            double disc = bCoef * bCoef - 4 * aCoef * cCoef;
            if (disc < 0) continue;
            disc = std::sqrt(disc);
            double t1 = (-bCoef - disc) / (2 * aCoef);
            double t2 = (-bCoef + disc) / (2 * aCoef);
            if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || cCoef < 0) return false;
        }
        return true;
    };

    std::vector<Point2D> result;
    result.push_back(path.front());
    size_t anchor = 0;
    for (size_t i = 1; i < path.size(); i++) {
        if (i == path.size() - 1) {
            result.push_back(path[i]);
            break;
        }
        if (!segmentClear(path[anchor], path[i + 1])) {
            result.push_back(path[i]);
            anchor = i;
        }
    }
    return result;
}
