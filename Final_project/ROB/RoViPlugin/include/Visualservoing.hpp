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
    Q calculateDeltaQ(std::vector<Vector2D<double> > uv, std::vector<Vector2D<double> > target, const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq);
    void robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f, Vector2D<double> *dUV, Vector2D<double> *uv);
    Jacobian combine_duv(std::vector<Vector2D<double> > duv_s);
    rw::math::Jacobian calc_img_jacb(std::vector<Vector2D<double> > uv, const double f, const double z);
    Jacobian combine_jacbs(std::vector<Jacobian> jacobians);


};

#endif // VISUALSERVOING_H
