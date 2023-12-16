#include <xsim/sdf_world.hpp>
#include "sdf_parser.hpp"

using namespace std ;
using namespace Eigen ;


namespace xsim {

SDFWorld SDFWorld::load(const std::string &fpath, const std::string &wn) {
    SDFWorld w ;
    SDFParser parser(w) ;
    parser.parse(fpath, wn) ;
    return w ;
}


}
