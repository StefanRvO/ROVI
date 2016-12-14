#include "Visualservoing.hpp"


VisualServoing::VisualServoing()
{

}

rw::math::Jacobian VisualServoing::calc_img_jacb(std::vector<Vector2D<double> > uv, const double f, const double z)
{
    std::vector<rw::math::Jacobian> jacbs;
    for(auto point : uv)
    {
        double u = point[0];
        double v = point[1];
        Jacobian img_jacb(2,6);
        img_jacb(0,0) = - f / z;
        img_jacb(0,1) = 0;
        img_jacb(0,2) = u / z;
        img_jacb(0,3) = u * v / f;
        img_jacb(0,4) = - (f * f + u * u) / f;
        img_jacb(0,5) = v;
        img_jacb(1,0) = 0;
        img_jacb(1,1) = - f / z;
        img_jacb(1,2) = v / z;
        img_jacb(1,3) = (f * f + v * v) / f;
        img_jacb(1,4) = - (u * v) / f;
        img_jacb(1,5) = - u;
        jacbs.push_back(img_jacb);
    }

    return combine_jacbs(jacbs);
}


Q VisualServoing::calculateDeltaQ(std::vector<Vector2D<double> > uv, std::vector<Vector2D<double> > target, const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq)
{
    if(uv.size() == 0) return Q(6,0,0,0,0,0,0);
    std::vector<Vector2D<double> > duv_s;
    duv_s.resize(uv.size());
    for(size_t i = 0; i < uv.size(); i++) duv_s[i] = target[i] - uv[i];

    // Calculate image jacobian
    Jacobian imageJacobian = calc_img_jacb(uv, f, z);

    // Calculate Zimage
    Jacobian zImage = imageJacobian * Sq * Jq;


    auto zImageT = zImage.e().transpose();

    auto tmp = (zImage.e() * zImageT).inverse();

    Jacobian duv_jac = combine_duv(duv_s);
    auto y = tmp * duv_jac.e();

    rw::math::Jacobian dq(zImageT * y);

    Q dq_q(dq.e());

    return dq_q;

}




Jacobian VisualServoing::combine_duv(std::vector<Vector2D<double> > duv_s)
{
    std::vector<Jacobian> jacbs;
    for(auto duv : duv_s)
        jacbs.push_back(Jacobian(duv.e()));
    return combine_jacbs(jacbs);
}



Jacobian VisualServoing::combine_jacbs(std::vector<Jacobian> jacobians)
{
    Eigen::MatrixXd C(0, jacobians[0].e().cols());
    for(auto jacb : jacobians)
    {
        Eigen::MatrixXd tmp(C.rows() + jacb.e().rows(), jacb.e().cols());
        tmp << C, jacb.e();
        C = tmp;
    }

    return Jacobian(C);
}
