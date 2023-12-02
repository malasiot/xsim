#include "player.hpp"

using namespace std ;
using namespace Eigen ;
using namespace xsim ;

std::string State::toString() const {
    switch ( type_ ) {

    case STATE_MAX_MOVES_REACHED:
        return "Max moves reached" ;
    case STATE_TARGET_REACHED:
        return "Target reached" ;
    case STATE_OBJECTS_TURNED:
        return "Objects turned" ;
    case STATE_OUTSIDE_OF_WORKSPACE:
        return "Outside of workspace" ;
    case STATE_UNREACHABLE:
        return "Unreachable" ;
    default:
        return "Valid state" ;
    }
}


struct Pt {
    Vector2f c_, n_ ;
};

Player::Player(const Parameters &params, World *world): params_(params), world_(world) {

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

void Player::reset() {
    world_->reset() ;
//    runSim(1.5) ;
    trial_ = 0 ;
}

tuple<State, bool> Player::step(const State &state, int64_t action_index) {
    const auto &action = actions_[action_index] ;

    const BoxState &bs = state.boxes_[action.box_id_];

    auto [p1, p2, pc] = computeMotion(Vector2f(bs.cx_, bs.cy_), -bs.theta_, action.loc_) ;

    Isometry3f orig ;
    float t1, t2 ;
    if ( !world_->plan(p1, p2, box_names_[action.box_id_], orig, t1, t2) ) {
        State new_state ;  // this should not happen
        new_state.boxes_ = state.boxes_ ;
        new_state.type_ = STATE_UNREACHABLE ;
        return make_tuple(new_state, true) ;
    }

    world_->execute(orig, t1, t2, 0.05) ;
    runSim(0.05) ;

    trial_ ++ ;

    State new_state = getState() ;

    if ( !new_state.isTerminal() ) world_->updateCollisionEnv() ;

    return make_tuple(new_state,  new_state.isTerminal()) ;
}

bool Player::isFeasible(const State &state, int64_t action_index) {
    const auto &action = actions_[action_index] ;

    const BoxState &bs = state.boxes_[action.box_id_] ;

    auto [p1, p2, pc] = computeMotion(Vector2f(bs.cx_, bs.cy_), -bs.theta_, action.loc_) ;

    Isometry3f orig ;
    float t1, t2 ;
    return world_->plan(p1, p2, box_names_[action.box_id_], orig, t1, t2) ;
}

std::vector<int64_t> Player::getFeasibleActions(const State &state) {
    std::vector<int64_t> act ;
    for( size_t i=0 ; i<actions_.size() ; i++ ) {
        if ( isFeasible(state, i) )
            act.push_back(i);
    }
    return act ;
}

State Player::getState() {
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

    state.feasible_ = getFeasibleActions(state);

    return state ;
}


void Player::runSim(float t) {
    float tt = 0 ;
    const float delta = 0.05 ;
    while ( tt < t ) {
        world_->stepSimulation(delta);
        tt += delta ;
    }
}

std::tuple<Vector3f, Vector3f, Vector3f> Player::computeMotion(const Vector2f &c, float theta, int action_id) {
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

Player::Parameters::Parameters(const cvx::Variant &config) {
    config.lookup("motion.start_offset", motion_start_offset_) ;
    config.lookup("motion.push_offset", motion_push_offset_) ;
    config.lookup("target.box", target_) ;
    config.lookup("max_trials", max_trials_) ;
    config.lookup("target.pos.x", target_pos_.x()) ;
    config.lookup("target.pos.y", target_pos_.y()) ;
    config.lookup("target.radius", target_radius_) ;
}
