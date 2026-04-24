#include "debugcollisioncheck.h"

#include "utils/common/WeldSeamInfo.h"

namespace {

bool rayQuadraticIntersection(const Eigen::Matrix2f& A, const Eigen::Vector2f& b, float c, float theta, float& dist, Eigen::Vector2f& xy) {
    Eigen::Vector2f dir(std::cos(theta), std::sin(theta));
    float qa = dir.dot(A * dir);
    float qb = b.dot(dir);

    dist = std::numeric_limits<float>::infinity();

    auto acceptRoot = [&](float r) {
        if (std::isfinite(r) && r >= 0.0f && r < dist) {
            dist = r;
            xy = r * dir;
        }
    };

    if (std::fabs(qa) < 1e-12f) {
        if (std::fabs(qb) < 1e-12f) {
            return false;
        }

        acceptRoot(-c / qb);
        return std::isfinite(dist);
    }

    float disc = qb * qb - 4.0f * qa * c;
    if (disc < -1e-5f) {
        return false;
    }

    disc = std::max(0.0f, disc);
    float sqrtDisc = std::sqrt(disc);
    float denom = 2.0f * qa;

    acceptRoot((-qb - sqrtDisc) / denom);
    acceptRoot((-qb + sqrtDisc) / denom);

    return std::isfinite(dist);
}

bool closestPointByRaySampling(const Eigen::Matrix2f& A, const Eigen::Vector2f& b, float c, float& bestDist, Eigen::Vector2f& bestXY) {
    constexpr float PI = 3.14159265358979323846f;
    constexpr int SAMPLE_COUNT = 720;

    float bestTheta = 0.0f;
    bool found = false;

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        float theta = 2.0f * PI * static_cast<float>(i) / static_cast<float>(SAMPLE_COUNT);
        float dist = std::numeric_limits<float>::infinity();
        Eigen::Vector2f xy;

        if (rayQuadraticIntersection(A, b, c, theta, dist, xy) && dist < bestDist) {
            bestDist = dist;
            bestXY = xy;
            bestTheta = theta;
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    float step = 2.0f * PI / static_cast<float>(SAMPLE_COUNT);
    for (int iter = 0; iter < 24; ++iter) {
        bool improved = false;

        for (int side = -1; side <= 1; side += 2) {
            float theta = bestTheta + static_cast<float>(side) * step;
            float dist = std::numeric_limits<float>::infinity();
            Eigen::Vector2f xy;

            if (rayQuadraticIntersection(A, b, c, theta, dist, xy) && dist < bestDist) {
                bestDist = dist;
                bestXY = xy;
                bestTheta = theta;
                improved = true;
            }
        }

        if (!improved) {
            step *= 0.5f;
        }
    }

    return true;
}

}  // namespace

debugCollisionCheck::debugCollisionCheck() {}
// ===============================
// 评估当前 offset 是否干涉
// ===============================
CollisionResult debugCollisionCheck::evalCollisionAtOffset(const Eigen::Vector3f& P, const Eigen::Vector3f& Z, float offset,
                                                           const Eigen::Vector3f& n_plane, float d_plane, const Eigen::Vector3f& cylC,
                                                           const Eigen::Vector3f& cylAxis, float cylRadius, float toolRadius) {
    CollisionResult out;

    Eigen::Vector3f base = P - offset * Z;
    Eigen::Vector3f toolN = Z.normalized();

    Eigen::Vector3f u, v;
    buildPlaneBasis(toolN, u, v);

    Eigen::Matrix<float, 3, 2> U;
    U.col(0) = u;
    U.col(1) = v;

    // ---------------- 平面 ----------------
    Eigen::Vector3f n_plane_norm = n_plane.normalized();
    Eigen::Vector2f a_line = U.transpose() * n_plane_norm;
    float c_line = n_plane_norm.dot(base) + d_plane;

    Eigen::Vector2f xyPlane;
    Eigen::Vector3f hitSec;
    out.distSec = closestPointOnLineInToolPlane(base, U, a_line, c_line, xyPlane, hitSec);
    out.hitSec = hitSec;

    // ---------------- 圆柱 ----------------
    Eigen::Vector3f a_cyl = cylAxis.normalized();
    Eigen::Vector3f d0 = base - cylC;

    Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - a_cyl * a_cyl.transpose();
    Eigen::Matrix2f A2 = U.transpose() * M * U;
    Eigen::Vector2f b2 = 2.0f * U.transpose() * M * d0;
    float c2 = d0.transpose() * M * d0 - cylRadius * cylRadius;

    Eigen::Vector2f xyCyl;
    Eigen::Vector3f hitMain;
    out.distMain = closestPointOnQuadraticInToolPlane(base, U, A2, b2, c2, xyCyl, hitMain);
    out.hitMain = hitMain;

    // ---------------- 干涉判断 ----------------
    bool isSec = std::isfinite(out.distSec) && (out.distSec < toolRadius);
    bool isMain = std::isfinite(out.distMain) && (out.distMain < toolRadius);
    out.isIntersect = isSec || isMain;

    return out;
}

// ===============================
// 找到最小安全 offset
// ===============================
float debugCollisionCheck::findSafeOffset(const Eigen::Vector3f& P, const Eigen::Vector3f& Z, float offset0, const Eigen::Vector3f& n_plane,
                                          float d_plane, const Eigen::Vector3f& cylC, const Eigen::Vector3f& cylAxis, float cylRadius,
                                          float toolRadius, float maxExtraOffset, float tolOffset) {
    CollisionResult c0 = evalCollisionAtOffset(P, Z, offset0, n_plane, d_plane, cylC, cylAxis, cylRadius, toolRadius);

    if (!c0.isIntersect) {
        return offset0;
    }

    float lo = offset0;
    float hi = offset0 + 1.0f;

    // 先扩展上界，直到不干涉
    while (hi <= offset0 + maxExtraOffset) {
        CollisionResult ch = evalCollisionAtOffset(P, Z, hi, n_plane, d_plane, cylC, cylAxis, cylRadius, toolRadius);

        if (!ch.isIntersect) {
            break;
        }

        lo = hi;
        hi = hi + std::max(1.0f, 0.5f * (hi - offset0));
    }

    if (hi > offset0 + maxExtraOffset) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // 二分搜索最小安全 offset
    for (int k = 0; k < 60; ++k) {
        if (std::fabs(hi - lo) < tolOffset) {
            break;
        }

        float mid = 0.5f * (lo + hi);
        CollisionResult cm = evalCollisionAtOffset(P, Z, mid, n_plane, d_plane, cylC, cylAxis, cylRadius, toolRadius);

        if (cm.isIntersect) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return hi;
}
CollisionResult debugCollisionCheck::evalCollisionTwoCylindersAtOffset(const Eigen::Vector3f& P, const Eigen::Vector3f& Z, float offset,
                                                                       const Eigen::Vector3f& cylC1, const Eigen::Vector3f& cylAxis1,
                                                                       float cylRadius1, const Eigen::Vector3f& cylC2,
                                                                       const Eigen::Vector3f& cylAxis2, float cylRadius2, float toolRadius) {
    CollisionResult out;

    Eigen::Vector3f base = P - offset * Z;
    Eigen::Vector3f toolN = Z.normalized();

    Eigen::Vector3f u, v;
    buildPlaneBasis(toolN, u, v);

    Eigen::Matrix<float, 3, 2> U;
    U.col(0) = u;
    U.col(1) = v;

    // ================= 圆柱1 =================
    {
        Eigen::Vector3f a_cyl = cylAxis1.normalized();
        Eigen::Vector3f d0 = base - cylC1;

        Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - a_cyl * a_cyl.transpose();

        Eigen::Matrix2f A2 = U.transpose() * M * U;
        Eigen::Vector2f b2 = 2.0f * U.transpose() * M * d0;
        float c2 = d0.transpose() * M * d0 - cylRadius1 * cylRadius1;

        Eigen::Vector2f xyCyl;
        Eigen::Vector3f hitMain;

        out.distMain = closestPointOnQuadraticInToolPlane(base, U, A2, b2, c2, xyCyl, hitMain);
        out.hitMain = hitMain;
    }

    // ================= 圆柱2 =================
    {
        Eigen::Vector3f a_cyl = cylAxis2.normalized();
        Eigen::Vector3f d0 = base - cylC2;

        Eigen::Matrix3f M = Eigen::Matrix3f::Identity() - a_cyl * a_cyl.transpose();

        Eigen::Matrix2f A2 = U.transpose() * M * U;
        Eigen::Vector2f b2 = 2.0f * U.transpose() * M * d0;
        float c2 = d0.transpose() * M * d0 - cylRadius2 * cylRadius2;

        Eigen::Vector2f xyCyl;
        Eigen::Vector3f hitSec;

        out.distSec = closestPointOnQuadraticInToolPlane(base, U, A2, b2, c2, xyCyl, hitSec);
        out.hitSec = hitSec;
    }

    bool isMain = std::isfinite(out.distMain) && out.distMain < toolRadius;
    bool isSec = std::isfinite(out.distSec) && out.distSec < toolRadius;

    out.isIntersect = isMain || isSec;
    return out;
}

float debugCollisionCheck::findSafeOffsetTwoCylinders(const Eigen::Vector3f& P, const Eigen::Vector3f& Z, float offset0, const Eigen::Vector3f& cylC1,
                                                      const Eigen::Vector3f& cylAxis1, float cylRadius1, const Eigen::Vector3f& cylC2,
                                                      const Eigen::Vector3f& cylAxis2, float cylRadius2, float toolRadius, float maxExtraOffset,
                                                      float tolOffset) {
    CollisionResult c0 = evalCollisionTwoCylindersAtOffset(P, Z, offset0, cylC1, cylAxis1, cylRadius1, cylC2, cylAxis2, cylRadius2, toolRadius);

    if (!c0.isIntersect) {
        return offset0;
    }

    float lo = offset0;
    float hi = offset0 + 1.0f;

    while (hi <= offset0 + maxExtraOffset) {
        CollisionResult ch = evalCollisionTwoCylindersAtOffset(P, Z, hi, cylC1, cylAxis1, cylRadius1, cylC2, cylAxis2, cylRadius2, toolRadius);

        if (!ch.isIntersect) {
            break;
        }

        lo = hi;
        hi = hi + std::max(1.0f, 0.5f * (hi - offset0));
    }

    if (hi > offset0 + maxExtraOffset) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    for (int k = 0; k < 60; ++k) {
        if (std::fabs(hi - lo) < tolOffset) {
            break;
        }

        float mid = 0.5f * (lo + hi);

        CollisionResult cm = evalCollisionTwoCylindersAtOffset(P, Z, mid, cylC1, cylAxis1, cylRadius1, cylC2, cylAxis2, cylRadius2, toolRadius);

        if (cm.isIntersect) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return hi;
}
// ===============================
// 建立工具平面基底
// ===============================
void debugCollisionCheck::buildPlaneBasis(const Eigen::Vector3f& n, Eigen::Vector3f& u, Eigen::Vector3f& v) {
    Eigen::Vector3f nn = n.normalized();

    Eigen::Vector3f tmp(1.0f, 0.0f, 0.0f);
    if (std::fabs(tmp.dot(nn)) > 0.9f) {
        tmp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }

    u = tmp.cross(nn);
    if (u.norm() < 1e-12f) {
        tmp = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        u = tmp.cross(nn);
    }

    u.normalize();
    v = nn.cross(u);
    v.normalize();
}

// ===============================
// 平面内：线到原点最近点
// line: a^T x + c = 0
// 返回 dist = ||xyBest||
// ===============================
float debugCollisionCheck::closestPointOnLineInToolPlane(const Eigen::Vector3f& base, const Eigen::Matrix<float, 3, 2>& U, const Eigen::Vector2f& a,
                                                         float c, Eigen::Vector2f& xyBest, Eigen::Vector3f& hitPoint) {
    if (a.norm() < 1e-12f) {
        xyBest.setZero();
        hitPoint = base;
        return std::fabs(c);
    }

    xyBest = -c * a / a.squaredNorm();
    hitPoint = base + U * xyBest;
    return xyBest.norm();
}

// ===============================
// 平面内：二次曲线最近点
// x^T A x + b^T x + c = 0
// 用拉格朗日乘子找最靠近原点的点
// ===============================
float debugCollisionCheck::closestPointOnQuadraticInToolPlane(const Eigen::Vector3f& base, const Eigen::Matrix<float, 3, 2>& U,
                                                              const Eigen::Matrix2f& A, const Eigen::Vector2f& b, float c, Eigen::Vector2f& xyBest,
                                                              Eigen::Vector3f& hitPoint) {
    Eigen::Matrix2f Asym = 0.5f * (A + A.transpose());
    std::vector<float> lambdaCand = collectLambdaCandidates(Asym, b, c);

    const float VAL_EPS = 1e-3f;

    float bestDist = std::numeric_limits<float>::infinity();

    Eigen::Vector2f bestXY = Eigen::Vector2f::Constant(std::numeric_limits<float>::quiet_NaN());

    for (float lam : lambdaCand) {
        Eigen::Matrix2f H = Eigen::Matrix2f::Identity() + lam * Asym;

        Eigen::FullPivLU<Eigen::Matrix2f> lu(H);
        if (!lu.isInvertible()) continue;

        Eigen::Vector2f xy = -(lam / 2.0f) * lu.solve(b);
        if (!xy.allFinite()) continue;

        float val = xy.dot(Asym * xy) + b.dot(xy) + c;
        if (!std::isfinite(val)) continue;

        if (std::fabs(val) > VAL_EPS) continue;

        float d = xy.norm();

        // 禁止 xy = 0 假解
        if (d < bestDist) {
            bestDist = d;
            bestXY = xy;
        }

        // 选择“最接近约束”的解（替代 fallback）
    }

    closestPointByRaySampling(Asym, b, c, bestDist, bestXY);

    if (!std::isfinite(bestDist)) {
        xyBest.setConstant(std::numeric_limits<float>::quiet_NaN());
        hitPoint.setConstant(std::numeric_limits<float>::quiet_NaN());
        return std::numeric_limits<float>::quiet_NaN();
    }

    xyBest = bestXY;
    hitPoint = base + U * xyBest;

    return bestDist;
}

// ===============================
// 收集 lambda 候选值
// ===============================
std::vector<float> debugCollisionCheck::collectLambdaCandidates(const Eigen::Matrix2f& A, const Eigen::Vector2f& b, float c) {
    std::vector<float> lambdaCand;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(A);
    Eigen::Vector2f eig = es.eigenvalues();

    std::vector<float> poles;
    for (int i = 0; i < 2; ++i) {
        if (std::fabs(eig[i]) > 1e-12f) {
            poles.push_back(-1.0f / eig[i]);
        }
    }

    std::sort(poles.begin(), poles.end());
    poles.erase(std::unique(poles.begin(), poles.end(), [](float a, float d) { return std::fabs(a - d) < 1e-8f; }), poles.end());

    const float L = 1e5f;

    std::vector<float> edges;
    edges.push_back(-L);
    for (float p : poles) {
        edges.push_back(p);
    }
    edges.push_back(L);

    std::sort(edges.begin(), edges.end());

    for (size_t k = 0; k + 1 < edges.size(); ++k) {
        float left = edges[k];
        float right = edges[k + 1];

        if (right - left < 1e-10f) {
            continue;
        }

        float pad = 1e-8f * std::max(1.0f, std::fabs(left) + std::fabs(right));
        float a = left + pad;
        float bnd = right - pad;

        if (a >= bnd) {
            continue;
        }

        const int N = 101;
        std::vector<float> sample(N);
        std::vector<float> vals(N);

        for (int i = 0; i < N; ++i) {
            sample[i] = a + (bnd - a) * static_cast<float>(i) / static_cast<float>(N - 1);
            vals[i] = quadraticStationaryValue(sample[i], A, b, c);
        }

        // 先收集接近 0 的点
        float minAbsVal = std::numeric_limits<float>::infinity();
        int minIdx = -1;
        for (int i = 0; i < N; ++i) {
            if (!std::isfinite(vals[i])) {
                continue;
            }

            float absVal = std::fabs(vals[i]);
            if (absVal < minAbsVal) {
                minAbsVal = absVal;
                minIdx = i;
            }
        }

        if (minIdx >= 0 && minAbsVal < 1e-6f) {
            lambdaCand.push_back(sample[minIdx]);
        }

        // 再收集符号变化的根
        for (int i = 0; i + 1 < N; ++i) {
            float v1 = vals[i];
            float v2 = vals[i + 1];

            if (!std::isfinite(v1) || !std::isfinite(v2)) {
                continue;
            }

            if (v1 == 0.0f) {
                lambdaCand.push_back(sample[i]);
            } else if (v1 * v2 < 0.0f) {
                float root = 0.0f;
                if (bisectRoot([&](float lam) { return quadraticStationaryValue(lam, A, b, c); }, sample[i], sample[i + 1], root)) {
                    lambdaCand.push_back(root);
                }
            }
        }
    }

    if (lambdaCand.empty()) {
        return lambdaCand;
    }

    std::sort(lambdaCand.begin(), lambdaCand.end());
    lambdaCand.erase(std::unique(lambdaCand.begin(), lambdaCand.end(), [](float x, float y) { return std::fabs(x - y) < 1e-6f; }), lambdaCand.end());

    return lambdaCand;
}

// ===============================
// 计算驻点方程值
// ===============================
float debugCollisionCheck::quadraticStationaryValue(float lambda, const Eigen::Matrix2f& A, const Eigen::Vector2f& b, float c) {
    Eigen::Matrix2f H = Eigen::Matrix2f::Identity() + lambda * A;

    Eigen::FullPivLU<Eigen::Matrix2f> lu(H);
    if (!lu.isInvertible()) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    Eigen::Vector2f x = -(lambda / 2.0f) * lu.solve(b);
    if (!x.allFinite()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    float val = x.dot(A * x) + b.dot(x) + c;
    return val;
}

// ===============================
// 二分求根
// ===============================
template <typename Func>
bool debugCollisionCheck::bisectRoot(Func f, float lo, float hi, float& root, int maxIter) {
    float fLo = f(lo);
    float fHi = f(hi);

    if (!std::isfinite(fLo) || !std::isfinite(fHi)) {
        return false;
    }

    if (fLo == 0.0f) {
        root = lo;
        return true;
    }
    if (fHi == 0.0f) {
        root = hi;
        return true;
    }
    if (fLo * fHi > 0.0f) {
        return false;
    }

    for (int i = 0; i < maxIter; ++i) {
        float mid = 0.5f * (lo + hi);
        float fMid = f(mid);

        if (!std::isfinite(fMid)) {
            return false;
        }

        if (std::fabs(fMid) < 1e-8f || std::fabs(hi - lo) < 1e-6f) {
            root = mid;
            return true;
        }

        if (fLo * fMid <= 0.0f) {
            hi = mid;
            fHi = fMid;
        } else {
            lo = mid;
            fLo = fMid;
        }
    }

    root = 0.5f * (lo + hi);
    return true;
}
