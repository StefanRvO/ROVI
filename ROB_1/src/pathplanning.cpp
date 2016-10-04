#include <iostream>
#include <fstream>
#include "findPathWithEpsilon.hpp"
#include "LuaGenerator.hpp"
using namespace std;


int main(int argc, char** argv)
{
	double epsilon = 0.1;
	findPathWithEpsilon test(argv[1], "KukaKr16");
	test.findPath(epsilon);
}
