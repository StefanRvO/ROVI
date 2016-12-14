#include "Visualservoing.hpp"


VisualServoing::VisualServoing()
{

}

rw::math::Jacobian VisualServoing::calculateImageJacobian(Vector2D<double> uv, const double f, const double z)
{
    rw::math::Jacobian imageJacobian(2,6);
    double u = uv[0];
    double v = uv[1];

    imageJacobian(0,0) = - f / z;
    imageJacobian(0,1) = 0;
    imageJacobian(0,2) = u / z;
    imageJacobian(0,3) = u * v / f;
    imageJacobian(0,4) = - (f * f + u * u) / f;
    imageJacobian(0,5) = v;
    imageJacobian(1,0) = 0;
    imageJacobian(1,1) = - f / z;
    imageJacobian(1,2) = v / z;
    imageJacobian(1,3) = (f * f + v * v) / f;
    imageJacobian(1,4) = - (u * v) / f;
    imageJacobian(1,5) = - u;

    return imageJacobian;
}

Jacobian VisualServoing::calculateImageJacobian(std::vector<double> uv, float f, float z)
{
    std::vector<Jacobian> jacobians;

    for (unsigned int i = 0; i < uv.size(); i += 2) {
        Jacobian imageJacobian(2,6);
        imageJacobian(0,0) = - f / z;
        imageJacobian(0,1) = 0;
        imageJacobian(0,2) = uv[i] / z;
        imageJacobian(0,3) = uv[i] * uv[i + 1] / f;
        imageJacobian(0,4) = - (f * f + uv[i] * uv[i]) / f;
        imageJacobian(0,5) = uv[i + 1];
        imageJacobian(1,0) = 0;
        imageJacobian(1,1) = - f / z;
        imageJacobian(1,2) = uv[i + 1] / z;
        imageJacobian(1,3) = (f * f + uv[i + 1] * uv[i + 1]) / f;
        imageJacobian(1,4) = - (uv[i] * uv[i + 1]) / f;
        imageJacobian(1,5) = - uv[i];
        jacobians.push_back(imageJacobian);
    }

    return mergeJacobians(jacobians);
}

Q VisualServoing::calculateDeltaQ(std::vector<Vector2D<double> > uv_, std::vector<Vector2D<double> > target_, const double z, const double f, rw::math::Jacobian Sq, rw::math::Jacobian Jq)
{
    std::vector<double> uv;
    for(auto point : uv_)
    {
        uv.push_back(point[0]);
        uv.push_back(point[1]);
    }
    std::vector<double> target;
    for(auto point : target_)
    {
        target.push_back(point[0]);
        target.push_back(point[1]);
    }
    last_uv.resize(uv.size());
    std::vector<double> duv;
    duv.resize(uv.size());
    for(size_t i = 0; i < uv.size(); i++) duv[i] = target[i] - uv[i];

    // Calculate image jacobian
    Jacobian imageJacobian = calculateImageJacobian(uv, f, z);

    // Calculate Zimage
    Jacobian zImage = imageJacobian * Sq * Jq;


    auto zImageT = zImage.e().transpose();

    auto tmp = (zImage.e() * zImageT).inverse();

    Jacobian duv_jac = vectorToJacobian(duv);
    auto y = tmp * duv_jac.e();

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
void VisualServoing::robotCoordToImageCoord(Vector3D<double> robotCoord, double z, double f, Vector2D<double> *dUV, Vector2D<double> *uv)
{
    double x = robotCoord[0];
    double y = robotCoord[1];
    Vector3D<double> cur_point(x,y,z);
    //std::cout << "cur\t" << cur_point << std::endl;
    auto delta_xyz = last_point - cur_point;
    last_point = cur_point;
    //std::cout << "delta\t" << delta_xyz << std::endl;


    double u = (f * x)/z;
    double v = (f * y)/z;
    *uv = Vector2D<double>(u,v);
    double du = (f/z) * delta_xyz[0] - ((f * x)/(z * z)) * delta_xyz[2];
    double dv = (f/z) * delta_xyz[1] - ((f * y)/(z * z)) * delta_xyz[2];
    *dUV = Vector2D<double>(du,dv);

    if(first_run)
    {
        first_run = false;
        *dUV = Vector2D<double>(0,0);
    }
    return;
}


Jacobian VisualServoing::vectorToJacobian(std::vector<double> vector)
{
    int n = vector.size();
    Jacobian jac(n,1);

    for (int i = 0; i < n; i++) {
        jac(i,0) = vector[i];
    }
    return jac;
}



Jacobian VisualServoing::mergeJacobians(std::vector<Jacobian> jacobians)
{
    int num = jacobians.size();
    int rows = jacobians[0].size1(), columns = jacobians[0].size2();

    Jacobian mergedJacobian(rows * num, columns);

    for (int i = 0; i < num; i++) {
        for (int row = 0; row < rows; row++) {
            for (int column = 0; column < columns; column++) {
                mergedJacobian(row + i * rows, column) = jacobians[i](row, column);
            }
        }
    }

    // for (size_t i = 0; i < mergedJacobian.size1(); i++) {
    //     for (size_t j = 0; j < mergedJacobian.size2(); j++) {
    //         std::cout << mergedJacobian(i,j) << '\t';
    //     }
    //     std::cout << '\n';
    // }
    // std::cout << '\n';

    return mergedJacobian;
}
