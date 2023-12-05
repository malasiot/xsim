#include "environment.hpp"
#include "robot.hpp"

using namespace std ;
using namespace Eigen ;
using namespace xsim ;

struct Pt {
    Vector2f c_, n_ ;
};

std::ostream &operator<<(std::ostream &strm, const State &state) {
    switch ( state.type_ ) {

    case STATE_MAX_MOVES_REACHED:
        strm << "Max moves reached" ;
        break ;
    case STATE_TARGET_REACHED:
        strm << "Target reached" ;
        break ;
    case STATE_OBJECTS_TURNED:
        strm << "Objects turned" ;
        break ;
    case STATE_OUTSIDE_OF_WORKSPACE:
        strm << "Outside of workspace" ;
        break ;
    case STATE_UNREACHABLE:
        strm << "Unreachable" ;
        break ;
    default:
        if ( state.target_moved_ )
          strm << "Target moved: " <<  state.movement_ <<';';
        if ( state.non_target_moved_ )
            strm << "Non target moved" ;
        break ;


    }

    return strm ;
}

Environment::Environment(const Parameters &params, World *world): params_(params), world_(world) {

    box_names_ = world_->getBoxNames() ;

    for( size_t j = 0 ; j<box_names_.size() ; j++ ) {
        const auto &b = box_names_[j] ;
        for( int i=0 ; i<12 ; i++ ) {
            PushAction a ;
            a.box_id_ = j ;
            a.loc_ = i ;
            actions_.emplace_back(a) ;
        }
        if ( b == params_.target_ )
            target_idx_ = j ;
    }


    runSim(0.5) ;
}

void Environment::reset() {
    world_->reset() ;
//    runSim(1.5) ;
    trial_ = 0 ;
}


void Environment::checkMotion(const State &old_state, State &new_state) {
    if ( new_state.isTerminal() ) return ;

    const Vector3f &box_sz = world_->params().box_sz_ ;

    for( size_t i=0 ; i<old_state.boxes_.size() ; i++ ) {
        const auto &box_state = old_state.boxes_[i] ;
        const auto &bname = box_state.name_ ;

        cv::RotatedRect rect1(cv::Point2f{box_state.cx_, box_state.cy_}, cv::Size2f{2*box_sz.x(), 2*box_sz.y()}, box_state.theta_ * 180/M_PI) ;

        const BoxState &new_box_state = new_state.boxes_[i] ;

        cv::RotatedRect rect2(cv::Point2f{new_box_state.cx_, new_box_state.cy_}, cv::Size2f{2*box_sz.x(), 2*box_sz.y()}, new_box_state.theta_ * 180/M_PI) ;

        cv::Point2f p1[4], p2[4] ;
        rect1.points(p1) ;
        rect2.points(p2) ;

        float dist = sqrt((p1[0] - p2[0]).dot(p1[0] - p2[0])) ;

        if ( dist > 0.001 ) {
            if ( bname == params_.target_ ) new_state.target_moved_ =  true ;
            else new_state.non_target_moved_ = true ;
        }

    }

    const auto &tc = params_.target_pos_ ;

    if ( new_state.target_moved_ ) {
        new_state.type_ = STATE_TARGET_MOVED ;
        auto &obox = old_state.boxes_[target_idx_] ;
        auto &nbox = new_state.boxes_[target_idx_] ;

        float dist1 = sqrt((obox.cx_ - tc.x())*(obox.cx_ - tc.x()) +
                           (obox.cy_ - tc.y())*(obox.cy_ - tc.y())) ;
        float dist2 = sqrt((nbox.cx_ - tc.x())*(nbox.cx_ - tc.x()) +
                           (nbox.cy_ - tc.y())*(nbox.cy_ - tc.y())) ;
        new_state.movement_ = dist1 - dist2 ;

    }



}

tuple<State, float, bool> Environment::transition(const State &state, int64_t action_index) {
    const auto &action = actions_[action_index] ;

    const BoxState &bs = state.boxes_[action.box_id_];

    auto [p1, p2, pc] = computeMotion(Vector2f(bs.cx_, bs.cy_), -bs.theta_, action.loc_) ;

    Isometry3f orig ;
    float t1, t2 ;
    if ( !world_->plan(p1, p2, box_names_[action.box_id_], orig, t1, t2) ) {
        State new_state ;  // this should not happen
        new_state.boxes_ = state.boxes_ ;
        new_state.type_ = STATE_UNREACHABLE ;
        return make_tuple(new_state, computeReward(new_state), true) ;
    }

    world_->execute(orig, t1, t2, 0.05) ;
    runSim(0.05) ;

    trial_ ++ ;

    State new_state = getState() ;

    const auto &tbox = new_state.boxes_[target_idx_] ;
    if ( Vector2f(tbox.cx_ - params_.target_pos_.x(), tbox.cy_ - params_.target_pos_.y()).norm() < params_.target_radius_ ) {
        new_state.type_ = STATE_TARGET_REACHED ;
        cout << "target reached" << endl ;
    }

    checkMotion(state, new_state) ;

    if ( !new_state.isTerminal() ) world_->updateCollisionEnv() ;

    return make_tuple(new_state, computeReward(new_state), new_state.isTerminal()) ;
}

bool Environment::isFeasible(const State &state, int64_t action_index) {
    const auto &action = actions_[action_index] ;

    const BoxState &bs = state.boxes_[action.box_id_] ;

    auto [p1, p2, pc] = computeMotion(Vector2f(bs.cx_, bs.cy_), -bs.theta_, action.loc_) ;

    Isometry3f orig ;
    float t1, t2 ;
    return world_->plan(p1, p2, box_names_[action.box_id_], orig, t1, t2) ;
}

State Environment::getState() const {
    State state ;

    if ( trial_ == params_.max_trials_ ) {
        state.type_ = STATE_MAX_MOVES_REACHED ;
    }

    state.boxes_.resize(box_names_.size()) ;

    const auto &bt = world_->getBoxTransforms() ;

    for( size_t i=0 ; i<box_names_.size() ; i++ ) {
        const string &name = box_names_[i] ;

        auto it = bt.find(name) ;
        assert( it != bt.end() ) ;

        Isometry3f tr = it->second;

        Vector3f top = tr.linear() * Vector3f(0, 0, world_->params().box_sz_.z()) ;
        if ( fabs(top.z()) < 0.01 ) {
            state.type_ = STATE_OBJECTS_TURNED ;
 //           break ;
        } else if ( tr.translation().z() < world_->params().box_sz_.z() - 0.01 ) {
            state.type_ = STATE_OUTSIDE_OF_WORKSPACE ;
//            break ;
        }

        Vector2f c = tr.translation().head<2>() ;

        BoxState &bs = state.boxes_[i] ;
        bs.cx_ = c.x() ;
        bs.cy_ = c.y() ;
        bs.name_ = name ;

        bs.theta_ = asin(tr(0, 1)) ;
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
    float bsx = world_->params().box_sz_.x(), bsy = world_->params().box_sz_.y() ;

    for( size_t i=0 ; i<state.boxes_.size() ; i++ ) {
        const BoxState &bs = state.boxes_[i] ;
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
    for( size_t i=0 ; i<state.boxes_.size() ; i++ ) {
        const BoxState &bs = state.boxes_[i] ;
        if ( i == a.box_id_ ) {
            center = Vector2f{bs.cx_, bs.cy_} ;
            theta = bs.theta_ ;
        }
        cv::Scalar color = ( i == a.box_id_ ) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255) ;
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

Eigen::VectorXf Environment::stateToTensor(const State &state) const {
    std::vector<float> state_vec;
    state_vec.reserve(state.boxes_.size() * 3);

    for ( size_t i=0 ; i<state.boxes_.size() ; i++ ){
        const auto &box = state.boxes_[i] ;
        state_vec.push_back(box.cx_ * 0.01);
        state_vec.push_back(box.cy_ * 0.01);
        state_vec.push_back(sin(box.theta_));
        state_vec.push_back(cos(box.theta_)) ;
    }

    return Map<VectorXf>(state_vec.data(), state_vec.size()) ;
//    at::Tensor state_tensor = at::from_blob(state_vec.data(), { 1, state_vec.size()}, at::TensorOptions().dtype(at::kFloat)).clone();

    //  return state_tensor;
}

VectorXf Environment::desiredGoal() const {
    VectorXf goal(2) ;
    goal[0] = params_.target_pos_.x() * 0.01 ;
    goal[1] = params_.target_pos_.y() * 0.01 ;

    return goal ;
}

VectorXf Environment::stateToGoal(const State &state) const {
    VectorXf goal(2) ;
    goal[0] = state.boxes_[target_idx_].cx_ * 0.01;
    goal[1] = state.boxes_[target_idx_].cy_ * 0.01;

    return goal ;
}

void Environment::runSim(float t) {
    float tt = 0 ;
    const float delta = 0.05 ;
    while ( tt < t ) {
        world_->stepSimulation(delta);
        tt += delta ;
    }
}

float Environment::computeReward(const State &state) {
    if ( state.type_ == STATE_OBJECTS_TURNED ) return -500 ;
    if ( state.type_ == STATE_OUTSIDE_OF_WORKSPACE ) return -500 ;
    if ( state.type_ == STATE_UNREACHABLE ) return -20 ;
    if ( state.type_ == STATE_MAX_MOVES_REACHED ) return -50 ;
    if ( state.type_ == STATE_TARGET_REACHED ) return 0 ;

    return -1 ;
    /*
    float r = 0 ;
    if ( state.non_target_moved_ ) r += -5 ;
    if ( state.target_moved_ )
        r += ( state.movement_ < 0 ? -2 : 1 ) ;
    return r ;
    */
}

float Environment::computeRewardForGoal(const State &state, const Eigen::VectorXf &goal) {
    const auto &tbox = state.boxes_[target_idx_] ;
    if ( Vector2f(tbox.cx_ - goal[0]/0.01, tbox.cy_ - goal[1]/0.01).norm() < params_.target_radius_ )
        return 0 ;
    else
        return -1 ;
}

std::tuple<Vector3f, Vector3f, Vector3f> Environment::computeMotion(const Vector2f &c, float theta, int action_id) {
    Vector3f hbox_size = world_->params().box_sz_ ;
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
    float start_delta = params_.motion_start_offset_ ;
    float motion_length = params_.motion_push_offset_ ;
    float height =  0.03 ;
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

Environment::Parameters::Parameters(const cvx::Variant &config) {
    config.lookup("motion.start_offset", motion_start_offset_) ;
    config.lookup("motion.push_offset", motion_push_offset_) ;
    config.lookup("target.pos.x", target_pos_.x()) ;
    config.lookup("target.pos.y", target_pos_.y()) ;
    config.lookup("target.radius", target_radius_) ;
    config.lookup("target.box", target_) ;
    config.lookup("max_trials", max_trials_) ;
}
