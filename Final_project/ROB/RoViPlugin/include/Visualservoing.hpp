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
    std::vector<double> last_uv;

    bool first_run = true;
    bool first_run_2 = true;

public:
    VisualServoing();
    rw::math::Jacobian calculateImageJacobian(Vector2D<double> dUV, const double f, const double z);
    Q calculateDeltaQ(std::vector<double> uv, std::vector<double> target, const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq);
    void robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f, Vector2D<double> *dUV, Vector2D<double> *uv);
    rw::math::Jacobian vectorToJacobian(std::vector<double> vector);
    Jacobian calculateImageJacobian(std::vector<double> uv, float f,  float z);
    Jacobian mergeJacobians(std::vector<Jacobian> jacobians);


};

#endif // VISUALSERVOING_H
