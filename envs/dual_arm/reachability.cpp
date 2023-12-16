#include <xsim/kinematic.hpp>
#include <xsim/collision_space.hpp>
#include <fstream>

#include "kuka_iiwa_ik_solver.hpp"

using namespace std ;
using namespace xsim ;
using namespace Eigen ;

static const float table_width = 3, table_height = 2.5 ;
static const float table_offset_x = 0.0, table_offset_y = 0.65 ;
static const double robot_dist = 0.72 ;
static const double minx = -1.0, maxx = 1.0 ;
static const double miny = 0.0, maxy = 1.0 ;
static const double minz = 0.0, maxz = 1.0 ;
static const double xstep = 0.05, ystep = 0.05, zstep = 0.05 ;

std::shared_ptr<CollisionSpace> collisions(new CollisionSpace()) ;
std::shared_ptr<KinematicModel> kinematics_r1, kinematics_r2 ;
Isometry3f r1_orig, r2_orig ;

void createScene(const std::string &model_path) {
    CollisionShapePtr table_cs(new BoxCollisionShape(Vector3f{table_width/2.0, table_height/2.0, 0.001})) ;
    table_cs->setMargin(0) ;
    Isometry3f table_tr(Translation3f{table_offset_x, table_offset_y,  -0.001}) ;


    CollisionShapePtr box_cs(new BoxCollisionShape({0.1, 0.1, 0.1})) ;
    box_cs->setMargin(0) ;
    Isometry3f box_tr(Translation3f{0, 0.1, 0.5}) ;


    r1_orig.setIdentity() ;
    r1_orig.translate(Vector3f{-robot_dist/2.0, 0, 0});


    r2_orig.setIdentity() ;
    r2_orig.translate(Vector3f{robot_dist/2.0, 0, 0});

    URDFRobot r1 = URDFRobot::load(model_path, "r1_") ;
    r1.setWorldTransform(r1_orig);

    kinematics_r1.reset(new KinematicModel(r1)) ;

    collisions->disableCollision("r1_link_0", "table");

    collisions->addRobot(r1, 0.01) ;

    URDFRobot r2 = URDFRobot::load(model_path, "r2_") ;
    r2.setWorldTransform(r2_orig);

    kinematics_r2.reset(new KinematicModel(r2)) ;

    collisions->disableCollision("r2_link_0", "table");

    collisions->addRobot(r2, 0.01) ;
    collisions->addCollisionObject("table", table_cs, table_tr);
//     collisions->addCollisionObject("box", box_cs, box_tr);
}

int main(int argc, char *argv[]) {
    // load URDFs
    string path = "/home/malasiot/source/xsim/data/" ;
    string model_path = path + "/robots/dual_arm_kuka_iiwa/iiwa.urdf";

    createScene(model_path) ;

 //   Matrix3f m1(AngleAxisf(M_PI/2, Vector3f::UnitY())) ;
   // Matrix3f m1(AngleAxisf(M_PI, Vector3f::UnitY())) ;
    Matrix3f m1(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;


    ofstream strm1("/tmp/reach1.txt") ;

    JointState state1 ;
    state1["r1_joint_a1"] = 0 ;
    state1["r1_joint_a2"] = 0 ;
    state1["r1_joint_a3"] = 0 ;
    state1["r1_joint_a4"] = 0 ;
    state1["r1_joint_a5"] = 0 ;
    state1["r1_joint_a6"] = 0 ;
    state1["r1_joint_a7"] = 0 ;

    for( double x = minx ; x <= maxx ; x+= xstep ) {
        for( double y = miny ; y <= maxy ; y+= ystep ) {
            for( double z = minz ; z <= maxz ; z+= zstep ) {
                Vector3f f(x + robot_dist/2, y, z) ;

                Isometry3f p = Isometry3f::Identity() ;
                p.translation() = f  ;
                p.linear() = m1 ;

                KukaIKSolver solver ;

                KukaIKSolver::Problem ik(p ) ;
                ik.setPsiSamples(24);

                string prefix("r1_") ;

                std::vector<JointCoeffs> solutions ;
                if ( solver.solve(ik,  solutions) ) {
                    for( const auto solution: solutions ) {
                        JointState state ;
                        for( uint j=0 ; j<7 ; j++ ) {
                            state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
                        }

                        kinematics_r1->setJointState(state) ;

                        map<string, Isometry3f> trs1 = kinematics_r1->getLinkTransforms() ;
                        collisions->updateObjectTransforms(trs1) ;

                        if ( !collisions->hasCollision() ) {
                            strm1 <<  x << ' ' << y << ' ' << z << endl ;
                            break ;
                        }
                    }
                }
            }
        }
    }


    kinematics_r1->setJointState(state1);
    collisions->updateObjectTransforms(kinematics_r1->getLinkTransforms()) ;


    //Matrix3f m2(AngleAxisf(-M_PI/2, Vector3f::UnitY())) ;
    //Matrix3f m2(AngleAxisf(M_PI, Vector3f::UnitY())) ;
    Matrix3f m2(AngleAxisf(-M_PI/2, Vector3f::UnitX())) ;

    ofstream strm2("/tmp/reach2.txt") ;

    for( double x = minx ; x <= maxx ; x+= xstep ) {
        for( double y = miny ; y <= maxy ; y+= ystep ) {
            for( double z = minz ; z <= maxz ; z+= zstep ) {
                Vector3f f(x - robot_dist/2, y, z) ;

                Isometry3f p = Isometry3f::Identity() ;
                p.translation() = f  ;
                p.linear() = m2 ;

                KukaIKSolver solver ;

                KukaIKSolver::Problem ik(p ) ;
                ik.setPsiSamples(24);

                string prefix("r2_") ;

                std::vector<JointCoeffs> solutions ;
                if ( solver.solve(ik,  solutions) ) {
                    for( const auto solution: solutions ) {
                        JointState state ;
                        for( uint j=0 ; j<7 ; j++ ) {
                            state.emplace(prefix + KukaIKSolver::s_joint_names[j], solution[j]) ;
                        }

                        kinematics_r2->setJointState(state) ;

                        map<string, Isometry3f> trs2 = kinematics_r2->getLinkTransforms() ;
                        collisions->updateObjectTransforms(trs2) ;

                        if ( !collisions->hasCollision() ) {
                            strm2 <<  x << ' ' << y << ' ' << z << endl ;
                            break ;
                        }
                    }
                }
            }
        }
    }

}
