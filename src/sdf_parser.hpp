#pragma once

#include <Eigen/Geometry>
#include <string>
#include <pugi/pugixml.hpp>

namespace xsim {

class SDFWorld ;
struct SDFModel ;
struct SDFLink ;
struct SDFJoint ;
struct SDFCollision ;
struct SDFVisual ;
struct SDFGeometry ;

class SDFParser {
public:
    SDFParser(SDFWorld &w): world_(w) {}
    void parse(const std::string &fpath, const std::string &wn) ;

private:

    bool parseWorld(pugi::xml_node &node, const std::string &wn) ;
    SDFModel parseModel(pugi::xml_node &node);
    SDFLink parseLink(pugi::xml_node &node);
    SDFJoint parseJoint(pugi::xml_node &node);
    SDFCollision parseCollision(pugi::xml_node &node);
    SDFVisual parseVisual(pugi::xml_node &node);
    SDFGeometry *parseGeometry(pugi::xml_node &node) ;

private:

    SDFWorld &world_ ;
    std::string path_, version_ ;

};


}
