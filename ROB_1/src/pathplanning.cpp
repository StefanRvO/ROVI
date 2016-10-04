#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include "findPathWithEpsilon.hpp"
#include "LuaGenerator.hpp"

using namespace std;
using namespace rw::math;

int main(int argc, char** argv)
{
	double epsilon = 0.1;
	Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
	Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);
	findPathWithEpsilon test(argv[1], "KukaKr16");

	QPath path = test.findPath(epsilon,from,to);
}
