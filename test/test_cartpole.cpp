#include <xviz/scene/renderer.hpp>
#include <xviz/scene/camera.hpp>
#include <xviz/scene/light.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/scene/material.hpp>
#include <xviz/scene/geometry.hpp>

#include <xviz/robot/robot_scene.hpp>

#include <xsim/kinematic.hpp>

#include <iostream>
#include <thread>
#include <xsim/world.hpp>

#include "bullet_gui.hpp"
#include <xsim/multi_body.hpp>

#include <QApplication>
#include <QMainWindow>

#include <xviz/gui/offscreen.hpp>

#include <chrono>
#include <random>

using namespace xviz ;
using namespace xsim ;
using namespace std ;
using namespace Eigen ;

std::random_device rd;
std::mt19937 gen(rd());

struct Environment {

    struct State {
        float pole_theta_, pole_vel_ ;
        float cart_pos_, cart_vel_ ;
    };

    enum Action {
        Left = 0 , Right = 1
    };

    CameraPtr cam_ ;
    unique_ptr<OffscreenRenderer> renderer_ ;

    Environment() {
        // create a camera
        uint width = 640, height = 480 ;
        PerspectiveCamera *pcam = new PerspectiveCamera(1, // aspect ratio
                                                        50*M_PI/180,   // fov
                                                        0.0001,        // zmin
                                                        10           // zmax
                                                        ) ;

        cam_.reset(pcam) ;
        cam_->setBgColor({1, 1, 1, 1});
        cam_->lookAt(Vector3f{0.0, 4, 0}, {0, 0, 0}, {0, 0, 1.0f}) ;
        cam_->setViewport(width, height)  ;

        // create the offscreen window

        renderer_.reset(new OffscreenRenderer(nullptr, QSize(width, height))) ;

    }

    void reset() {

        physics_.reset(new PhysicsWorld) ;
        physics_->createMultiBodyDynamicsWorld();
        physics_->setGravity({0, 0, -10});

        auto scene = physics_->getVisual() ;

        string path = "/home/malasiot/local/bullet3/examples/pybullet/gym/pybullet_data/cartpole.urdf" ;

        auto robot = URDFRobot::load(path) ;

        DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
        dl->diffuse_color_ = Vector3f(1, 1, 1) ;
        scene->addLightNode(LightPtr(dl)) ;

        MultiBodyBuilder mb ;
        mb.loadURDF(robot) ;

        MultiBodyPtr body = physics_->addMultiBody(mb);

        jcp_ = body->findJoint("cart_to_pole") ;
        jsc_ = body->findJoint("slider_to_cart") ;
        jcp_->setMotorMaxImpulse(0.0) ;
        jsc_->setMotorMaxImpulse(0.0) ;
     //   jcp_->setDamping(0.0) ;
     //   jsc_->setDamping(0.0) ;
    //    jcp_->setFriction(0.0) ;
     //   jsc_->setFriction(0.0) ;


        vector<float> rp(4) ;
        std::uniform_real_distribution<> dis(-0.05f, 0.05f);
        for( uint i=0 ; i<4 ; i++ )
            rp[i] = dis(gen) ;

        jcp_->setPosition(rp[0]) ;
        jcp_->setVelocity(rp[1]) ;
        jsc_->setPosition(rp[2]) ;
        jsc_->setVelocity(rp[3]) ;
        jsc_->setTorque(0.0);

    }

    State getState() {
        State s ;
        s.pole_theta_ = jcp_->getPosition() ;
        s.pole_vel_ = jcp_->getVelocity() ;
        s.cart_pos_ = jsc_->getPosition() ;
        s.cart_vel_ = jsc_->getVelocity() ;
        return s;
    }

    QImage render() {
        return renderer_->render(physics_->getVisual(), cam_) ;
    }

    const float action_force = 1.f ;
    const float time_step = 0.02f ;
    const float x_threshold = 0.4f ;
    const float theta_threshold_radians = 12 * 2 * M_PI / 360;
    const float linear_velocity_limit = 10.f ;
    const float angular_velocity_limit = 10.f ;
    const int pos_bins = 10 ;
    const int theta_bins = 10 ;
    const float linear_velocity_bins = 10 ;
    const float angular_velocity_bins = 10 ;

    void step(int a, State &new_state, float &reward, bool &done) {
        float force ;
        switch ( a ) {
        case Action::Left:
            force = -action_force ;
            break ;
        case Action::Right:
            force = action_force ;
        }

        jcp_->setTorque(force) ;
        physics_->stepSimulation(time_step) ;

        new_state = getState() ;
        done =  new_state.cart_pos_ < -x_threshold || new_state.cart_pos_ > x_threshold ||
                       new_state.pole_theta_ < -theta_threshold_radians ||
                       new_state.pole_theta_ > theta_threshold_radians;

        reward = 1.0f;
    }

    uint32_t quantizeState(const State &s) {
        uchar cpos_bin = floor(pos_bins * (s.cart_pos_ - ( -x_threshold ) )/(2 * x_threshold)) ;
        uchar cvel_bin = floor(linear_velocity_bins * ( s.cart_vel_ - ( -linear_velocity_limit )/(2 * linear_velocity_limit))) ;
        uchar ptheta_bin = floor(theta_bins * (s.pole_theta_ - ( -theta_threshold_radians ) )/(2 * theta_threshold_radians)) ;
        uchar pvel_bin = floor(angular_velocity_bins * ( s.pole_vel_ - ( -angular_velocity_limit )/(2 * angular_velocity_limit)));

       return ((cpos_bin << 24) | (cvel_bin << 16) | (ptheta_bin << 8) | pvel_bin ) ;
    }

    int numActions() const { return 2 ; }

    std::unique_ptr<PhysicsWorld> physics_ ;
    ScenePtr scene_{new Scene()} ;
    uint body_idx_ ;
    Joint *jcp_, *jsc_ ;
};

struct Agent {

    Agent(Environment &env): env_(env) {}

    float getQValue(uint32_t state, int action) {
        auto it = Q_.find(state) ;
        if ( it == Q_.end() ) return 0.0 ;
        auto &p = it->second ;
        if ( action >= p.size() ) return 0.0 ;
        return p[action] ;
    }

    void setQValue(uint32_t state, int action, float val) {
        auto it = Q_.find(state) ;
        if ( it == Q_.end() )
            Q_[state].resize(env_.numActions(), 0.0f) ;
        Q_[state][action] = val ;
    }

    void train(int num_episodes, float alpha, float epsilon, float gamma) {

        int end_epsilon_decay = num_episodes/2 ;
        float epsilon_decay_step = epsilon/num_episodes ;

        for ( uint ep = 0 ; ep<num_episodes ; ep++ ) {
            env_.reset() ;
            Environment::State state = env_.getState() ;
            bool done = false ;
            float reward ;
            uint step ;



            float avg_reward = 0.0 ;

            for( step = 0 ; step < 200 && !done ; step ++ ) {

                uint32_t q_state = env_.quantizeState(state) ;

                          int action = chooseAction(q_state, epsilon) ;

                env_.step(action, state, reward, done) ;


                auto it = Q_.find(q_state) ;
                if ( it == Q_.end() ) {
                    Q_[q_state].resize(env_.numActions(), 0.0) ;
                    Q_[q_state][action] = reward ;
                } else {

                    uint32_t new_q_state = env_.quantizeState(state);


                    float old_value = getQValue(q_state, action) ;

                    float next_max = getMaxQ(new_q_state) ;

                    float new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max) ;
                    setQValue(q_state, action, new_value) ;
                }

                if ( ep % callback_freq_ == 0 ) {
                    callback_() ;
                }

                avg_reward += reward ;

            }

        //    if ( ep <= end_epsilon_decay)
         //       epsilon -= epsilon_decay_step ;

            cout << ep << ' ' << step << endl ;

        }
    }

    void setCallback(uint freq, std::function<void()> callback) {
        callback_ = callback ;
        callback_freq_ = freq ;
    }

    int chooseAction(uint32_t state, float epsilon) {

        auto it = Q_.find(state) ;

        std::uniform_real_distribution<> dis(0.0, 1.0) ;
        std::uniform_int_distribution<> action_dis(0, env_.numActions()-1) ;
        if ( it == Q_.end() || dis(gen) < epsilon ) {
            return action_dis(gen) ;
        }
        else {
            assert(it->second.size() == 2) ;
            auto ele = std::max_element(begin(it->second), end(it->second));
            int action = std::distance(begin(it->second), ele);
            return action ;
        }

    }


    float getMaxQ(uint32_t state) {
        auto it = Q_.find(state) ;
        if ( it == Q_.end() ) return 0.0 ;
        const auto &p = it->second ;
        return *std::max_element(p.begin(), p.end());

    }

    void updateQ(uint32_t state, int action,
                 float prev_reward, uint32_t prev_state, int prev_action, float alpha, float gamma) {
        Q_[prev_state][prev_action] = (1 - alpha) *
                   Q_[prev_state][prev_action] + alpha *
                   (prev_reward + (gamma * getMaxQ(state)));
    }

    map<uint32_t, vector<float>> Q_ ;
    float prev_reward_ ;
    float alpha_, gamma_ ;
    uint32_t prev_q_state_ ;
    int prev_action_ ;
    std::function<void()> callback_ ;
    uint callback_freq_ = 10 ;
    Environment &env_ ;
};


int main(int argc, char **argv)
{

    QApplication app(argc, argv) ;
    QMainWindow win ;

    SceneViewer::initDefaultGLContext();
    ImageWidget *iw = new ImageWidget(&win) ;
    win.setCentralWidget(iw) ;
    win.resize(800, 600) ;
    win.show() ;

    thread t( [&]{
        Environment env ;
        Agent agent(env) ;

        agent.setCallback(500, [&] {
            auto im = env.render() ;
            im.save("/tmp/oo.png");
            QMetaObject::invokeMethod(iw, "setImage", Qt::QueuedConnection, Q_ARG(QImage, im) );
        });

        agent.train(10000, 0.9, 0.1, 0.9);
    }) ;


    app.exec() ;

}
