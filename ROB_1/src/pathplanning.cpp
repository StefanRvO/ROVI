#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include "findPathWithEpsilon.hpp"
#include "LuaGenerator.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace rw::math;

int main(int argc, char** argv)
{
	double epsilon = 0.001;
	Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
	Q to(6,1.571, 0.006, 0.03, 0.153, 0.762, 4.49);

	findPathWithEpsilon test(argv[1], "KukaKr16");

	QPath path = test.findPath(epsilon,from,to);
    std::cout << get_path_length(path) << std::endl;
    std::cout << argc << std::endl;
    if(argc > 2 and std::string(argv[2]) == std::string("-getlua"))
    {
        LuaGenerator lua;
        lua.make_preample();
        lua.add_start_point(from);
        for(auto &q : path) lua.add_point(q);
        lua.add_end();
        std::cout << lua.get_string();
    }
    return 0;
}
