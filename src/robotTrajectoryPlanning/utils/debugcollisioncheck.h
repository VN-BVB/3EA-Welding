#ifndef DEBUGCOLLISIONCHECK_H
#define DEBUGCOLLISIONCHECK_H
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
struct CollisionResult;
class debugCollisionCheck {
public:
    debugCollisionCheck();
    CollisionResult evalCollisionAtOffset(const Eigen::Vector3f &P, const Eigen::Vector3f &Z, float offset, const Eigen::Vector3f &n_plane,
                                          float d_plane, const Eigen::Vector3f &cylC, const Eigen::Vector3f &cylAxis, float cylRadius,
                                          float toolRadius);
    float findSafeOffset(const Eigen::Vector3f &P, const Eigen::Vector3f &Z, float offset0, const Eigen::Vector3f &n_plane, float d_plane,
                         const Eigen::Vector3f &cylC, const Eigen::Vector3f &cylAxis, float cylRadius, float toolRadius, float maxExtraOffset,
                         float tolOffset);
    void buildPlaneBasis(const Eigen::Vector3f &n, Eigen::Vector3f &u, Eigen::Vector3f &v);
    float closestPointOnLineInToolPlane(const Eigen::Vector3f &base, const Eigen::Matrix<float, 3, 2> &U, const Eigen::Vector2f &a, float c,
                                        Eigen::Vector2f &xyBest, Eigen::Vector3f &hitPoint);
    float closestPointOnQuadraticInToolPlane(const Eigen::Vector3f &base, const Eigen::Matrix<float, 3, 2> &U, const Eigen::Matrix2f &A,
                                             const Eigen::Vector2f &b, float c, Eigen::Vector2f &xyBest, Eigen::Vector3f &hitPoint);
    std::vector<float> collectLambdaCandidates(const Eigen::Matrix2f &A, const Eigen::Vector2f &b, float c);
    float quadraticStationaryValue(float lambda, const Eigen::Matrix2f &A, const Eigen::Vector2f &b, float c);
    template <typename Func>
    bool bisectRoot(Func f, float lo, float hi, float &root, int maxIter = 80);
};

#endif  // DEBUGCOLLISIONCHECK_H
