#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include <rw/math/Jacobian.hpp>
#include <rw/math/Vector2D.hpp>

using namespace rw::math;


class VisualServoing
{
public:
    VisualServoing();
    rw::math::Jacobian calculateImageJacobian(const int u, const int v, const int f, const int z);
    void calculateDeltaQ(const int u, const int v, const int z, const int f, rw::math::Jacobian Sq, rw::math::Jacobian Jq);
    Vector2D<double> robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f);
};

#endif // VISUALSERVOING_H
