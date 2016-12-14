#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include <rw/math/Jacobian.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/LinearAlgebra.hpp>

using namespace rw::math;



class VisualServoing
{
private:
    Vector3D<double> last_point;
    bool first_run = true;

public:
    VisualServoing();
    rw::math::Jacobian calculateImageJacobian(Vector2D<double> dUV, const double f, const double z);
    Q calculateDeltaQ(Vector2D<double> dUV,Vector2D<double> uv , const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq);
    void robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f, Vector2D<double> *dUV, Vector2D<double> *uv);
};

#endif // VISUALSERVOING_H
