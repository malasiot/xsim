#include "environment.hpp"
#include "robot.hpp"

using namespace std ;
using namespace Eigen ;
using namespace xsim ;

struct Pt {
    Vector2f c_, n_ ;
};

Environment::Environment(World *world): world_(world) {
    runSim(0.5) ;
}

void Environment::reset() {
    world_->reset() ;
}

std::vector<PushAction> Environment::getActions() const {
    vector<PushAction> actions ;
    for( const auto &b: getBoxNames() ) {
        for( int i=0 ; i<12 ; i++ ) {
            PushAction a ;
            a.box_id_ = b ;
            a.loc_ = i ;
            actions.emplace_back(a) ;
        }
    }
}

float Environment::apply(const State &state, const PushAction &action) {


    auto it = state.boxes_.find(action.box_id_) ;
    assert ( it != state.boxes_.end() ) ;
    const BoxState &bs = it->second ;

    auto [p1, p2, pc] = computeMotion(Vector2f(bs.cx_, bs.cy_), -bs.theta_, action.loc_) ;
    cout << p1.adjoint() << ' ' <<p2.adjoint() << endl ;

    Matrix3f rot(AngleAxisf(M_PI, Vector3f::UnitY())) ;
    Isometry3f tr = Isometry3f::Identity() ;
    tr.translation() = p1  ;
    tr.linear() = rot.matrix() ;

    JointTrajectory traj ;
    if ( !world_->controller_->plan(tr, traj) ) {
        return false ;
    }

//    world_->controller_->executeTrajectory(*world_, traj, 5.0) ;

    world_->controller_->setJointState(traj.points().back()) ;

    world_->disableToolCollisions(action.box_id_) ;

    JointTrajectory push ;
    if ( !world_->controller_->planRelative(p2 - p1, push) ) {
        return false ;
    }

    world_->controller_->executeTrajectory(*world_, push, 0.01) ;

    world_->enableToolCollisions(action.box_id_) ;

    world_->resetRobot() ;

    runSim(0.5) ;

    return true ;
}

State Environment::getState() const {
    State state ;
    for( const auto &b: world_->boxes_ ) {
        string name = b->getName() ;
        Isometry3f tr = b->getWorldTransform();
        Vector2f c = tr.translation().head<2>() ;
        BoxState bs ;
        bs.cx_ = c.x() ;
        bs.cy_ = c.y() ;
        bs.theta_ = acos(tr(0, 0)) ;

        cout << name << ' ' << c.adjoint() << ' ' << bs.theta_ << endl ;
        state.boxes_.emplace(name, std::move(bs)) ;
    }

    return state ;
}

void draw_rotated_rect(cv::Mat& image, const cv::Point2f &center, const cv::Size2f &size, float rot, const cv::Scalar &color)
{
    // Create the rotated rectangle
    cv::RotatedRect rotated(center, size, rot * 180/M_PI);

    // We take the edges that OpenCV calculated for us
    cv::Point2f vert[4];
    rotated.points(vert);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i){
        vertices[i] = vert[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image, vertices, 4, color);
}

void draw_arrow(cv::Mat &im, const cv::Point &p1, const cv::Point &p2) {
    cv::line(im, p1, p2, cv::Scalar(255, 255, 0), 2) ;
    cv::circle(im, p1, 4, cv::Scalar(255, 0, 255)) ;
}

cv::Mat Environment::renderState(const PushAction &a, const State &state) {
    const float large = std::numeric_limits<float>::max() ;
    const float scale = 2000 ;
    float minx = large, miny = large ;
    float maxx = -large, maxy = -large ;
    float bsx = world_->params_.box_sz_.x(), bsy = world_->params_.box_sz_.y() ;

    for( const auto &bp: state.boxes_ ) {
        const auto &bs = bp.second ;
        minx = std::min(minx, bs.cx_ - 2*bsx) ;
        miny = std::min(miny, bs.cy_ - 2*bsy) ;
        maxx = std::max(maxx, bs.cx_ + 2*bsx) ;
        maxy = std::max(maxy, bs.cy_ + 2*bsy) ;
    }

    int width = ceil(scale*maxx) - floor(scale*minx) +1 ;
    int height = ceil(scale*maxy) - floor(scale*miny) +1 ;

    cv::Mat im(height, width, CV_8UC3, cv::Scalar(0, 0, 0)) ;

    Vector2f center ;
    float theta ;
    for( const auto &bp: state.boxes_ ) {
        const auto &bs = bp.second ;
        if ( bp.first == a.box_id_ ) {
            center = Vector2f{bs.cx_, bs.cy_} ;
            theta = bs.theta_ ;
        }
        cv::Scalar color = ( bp.first == a.box_id_ ) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255) ;
        cv::Point2f c(scale*(bs.cx_ - minx), scale*(bs.cy_ - miny)) ;
        cv::Size2f  s(2*scale*bsx, 2*scale*bsy) ;
        draw_rotated_rect(im, c, s, -bs.theta_, color) ;
     }

    auto [p1, p2, pc] = computeMotion(center, -theta, a.loc_) ;

    draw_arrow(im, cv::Point(scale*(p1.x() - minx), scale*(p1.y() - miny)),
               cv::Point(scale*(p2.x() - minx), scale*(p2.y() - miny)));


    cv::flip(im, im, 0) ;
    return im ;
}

std::vector<string> Environment::getBoxNames() const {
    vector<string> out ;
    for( const auto &b: world_->boxes_ ) {
        out.push_back(b->getName()) ;
    }
    return out ;
}

void Environment::runSim(float t) {
    float tt = 0 ;
    const float delta = 0.05 ;
    while ( tt < t ) {
        world_->stepSimulation(delta);
        tt += delta ;
    }
}

std::tuple<Vector3f, Vector3f, Vector3f> Environment::computeMotion(const Vector2f &c, float theta, int action_id) {
    Vector3f hbox_size = world_->params_.box_sz_ ;
    float w = hbox_size.x(), h = hbox_size.y() ;
    const vector<Pt> coords = {        { { w, -2 * h /3.0 }, { 1, 0} },
                                       { { w,  0          }, { 1, 0} },
                                       { { w,  2 * h /3.0 }, { 1, 0} },
                                       { {-w, -2 * h /3.0 }, {-1, 0} },
                                       { {-w,  0          }, {-1, 0} },
                                       { {-w,  2 * h/3.0  }, {-1, 0} },
                                       { {-2 * w/3.0,   h }, { 0, 1} },
                                       { { 0,           h }, { 0, 1} },
                                       { { 2 * w/3.0,   h }, { 0, 1} },
                                       { {-2 * w/3.0,  -h }, {0, -1} },
                                       { { 0,          -h }, {0, -1} },
                                       { { 2 * w/3.0,  -h }, {0, -1} }
                                     };
    float start_delta = world_->params_.motion_start_offset_ ;
    float motion_length = world_->params_.motion_push_offset_ ;
    float height =  0.02 ;
  //  float height = hbox_size.z();
    const Pt &cc = coords[action_id] ;
    Rotation2Df r(theta) ;
    Vector2f c0 = r * cc.c_, dir = r * cc.n_ ;
    Vector2f center = c0 + c ;
    Vector2f start = center + dir * start_delta ;
    Vector2f stop = center - dir * motion_length ;

    Vector3f p1(start.x(), start.y(), height), p2(stop.x(), stop.y(), height), p3(center.x(), center.y(), hbox_size.z()) ;

    return make_tuple(p1, p2, p3) ;
}
