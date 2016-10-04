#include <iostream>
#include <fstream>
#include "findPathWithEpsilon.hpp"

using namespace std;

void createLuaFile(QPath &path)
{
	ofstream file;
  file.open ("LUA.txt");
  file << "Test";
  file.close();
}

int main(int argc, char** argv)
{
	double epsilon = 0.1;
	findPathWithEpsilon test(argv[1], "KukaKr16");

	test.findPath(epsilon);

	//createLuaFile(path);
}
