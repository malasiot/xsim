#include <xsim/sdf_world.hpp>

#include <iostream>

using namespace xsim ;
using namespace std ;

int main(int argc, char *argv[]) {
    SDFWorld w = SDFWorld::load("/home/malasiot/local/bullet3/data/kuka_iiwa/kuka_world.sdf") ;

    cout << "ok" << endl ;
}
