#include "Visualservoing.hpp"


VisualServoing::VisualServoing()
{

}

rw::math::Jacobian VisualServoing::calculateImageJacobian(const int u, const int v, const int f, const int z)
{
    rw::math::Jacobian imageJacobian(2,6);

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

void VisualServoing::calculateDeltaQ(const int u, const int v, const int z, const int f, rw::math::Jacobian Sq, rw::math::Jacobian Jq)
{
    rw::math::Jacobian imageJacobian = calculateImageJacobian(u, v, f, z);

    rw::math::Jacobian zImage = imageJacobian * Sq * Jq;
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
