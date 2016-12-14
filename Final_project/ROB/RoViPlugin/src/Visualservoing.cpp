#include "Visualservoing.hpp"


VisualServoing::VisualServoing()
{

}

rw::math::Jacobian VisualServoing::calculateImageJacobian(Vector2D<double> uv, const double f, const double z)
{
    rw::math::Jacobian imageJacobian(2,6);
    double u = uv[0];
    double v = uv[1];

    imageJacobian(0,0) = -f/z;
    imageJacobian(0,1) = 0;
    imageJacobian(0,2) = u/z;
    imageJacobian(0,3) = (u*v)/f;
    imageJacobian(0,4) = -(f*f*+u*u)/f;
    imageJacobian(0,5) = v;
    imageJacobian(1,0) = 0;
    imageJacobian(1,1) = -f/z;
    imageJacobian(1,2) = v/z;
    imageJacobian(1,3) = (f*f*+u*u)/f;
    imageJacobian(1,4) = -(u*v)/f;
    imageJacobian(1,5) = -u;

    return imageJacobian;
}

Q VisualServoing::calculateDeltaQ(Vector2D<double> uv, const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq)
{
    // Calculate du and dv
    Vector2D<double>dUV(0,0);
    if(first_run)
    {
        first_run = false;
    }
    else
    {
        dUV = uv - lastUV;
    }

    lastUV = uv;
    std::cout << "duv: " << dUV << std::endl;


    Jacobian duvJacobian(dUV.e());

    // Calculate image jacobian
    Jacobian imageJacobian = calculateImageJacobian(uv, f, z);

    // Calculate Zimage
    Jacobian zImage = imageJacobian * Sq * Jq;


    auto zImageT = zImage.e().transpose();

    auto tmp = (zImage.e() * zImageT).inverse();


    auto y = (tmp * duvJacobian.e());

    rw::math::Jacobian dq(zImageT * y);

    Q dq_q(dq.e());

    //std::cout << dq_q << std::endl;

    return dq_q;

    //auto tmp1 = tmp * dUV.e();
   // rw::math::LinearAlgebra::EigenMatrix<double> test = (zImage * zImage.e().transpose()).e().inverse();
   // LinearAlgebra::EigenMatrix<double> test = (zImage * .e().inverse();
    //auto test1 = test *dUV;
    
}


/*
*   Converts a coordinate seen in from the robot camera 3D to a image coordinate in 2D
*/
Vector2D<double> VisualServoing::robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f)
{
    double x = robotCoord[1];
    double y = robotCoord[2];
    //double z = robotCoord[3];

    double u = (f*x) / z;
    double v = (f*y) / z;

    return Vector2D<double>(u,v);
}

