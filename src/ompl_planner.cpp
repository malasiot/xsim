#include <xsim/ompl_planner.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace std ;
using namespace Eigen ;

namespace xsim {

void JointTrajectory::append(const JointTrajectory &other)
{
    for( int i=1 ; i<other.points().size() ; i++ ) {
        const auto &js = other.points()[i] ;
        addPoint(0, js) ;
    }
}

JointState JointTrajectory::getState(float t, PlanningInterface *iplan) const{
    t = std::max(0.f, t) ;
    t = std::min(1.f, t) ;

    int idx = -1 ;
    for( int i=1 ; i<points().size() ; i++ ) {
        if ( times_[i] >= t ) {
            idx = i-1 ;
            t = t - times_[idx] ;
            break ;
        }
    }

    assert ( idx >= 0 ) ;

    const auto &chain = iplan->getJointChain() ;

    t /= (times_[idx+1] - times_[idx]) ;
//    cout << idx << ' ' << t << endl ;

    const JointState &p1 = points_[idx], &p2 = points_[idx+1] ;

    JointState sample ;
    for( int i=0 ; i<chain.size() ; i++ ) {
        const auto &jn = chain[i] ;
        auto i1 = p1.find(jn), i2 = p2.find(jn) ;
        float v1 = i1->second, v2 = i2->second ;

        float val = ( 1.0f - t ) * v1 + t * v2 ;

   //     cout << jn << ":  " << v1 << ' ' << v2 << ' ' << val << endl ;
        sample.emplace(jn, val) ;
    }

    return sample ;


}



}
