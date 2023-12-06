#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "world.hpp"
#include "player.hpp"

#include <QTimer>
#include <QThread>
#include <QTcpSocket>

#include <iostream>

#include <xviz/gui/manipulator.hpp>

class QTcpServer ;

Q_DECLARE_METATYPE(State)

class ExecuteStepThread: public QThread {

    Q_OBJECT
public:
    ExecuteStepThread(Player *env, int64_t action): env_(env), action_id_(action) {
        env_->world()->setUpdateCallback([this]() {
            emit updateScene();
        });
    }

    ~ExecuteStepThread() {
        env_->world()->setUpdateCallback(nullptr);
    }

    void run() override {
        auto state = env_->getState() ;
        auto [new_state,  done] = env_->step(state, action_id_) ;
        emit finishedStep(new_state, done) ;
    }

signals:
    void updateScene();
    void finishedStep(const State &state, bool done) ;
protected:

    Player *env_ ;
    int64_t action_id_ ;

};

class GUI: public SimulationGui {
    Q_OBJECT
public:
    GUI(World *p);

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;
    void setGrabFramePath(const QString &path) {
        grab_frame_path_ = path ;
    }

    void onUpdate(float delta) override;


   void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

private slots:
    void grabScreen();
    void startRecording() ;
    void stopRecording() ;
signals:
    void recordingStarted() ;
    void recoringStopped() ;
private:


    xviz::NodePtr target_ ;
    World *world_ ;
    std::shared_ptr<xviz::TransformManipulator> gizmo_;

    QTimer *timer_ ;
    int64_t count_ ;
    QString grab_frame_path_, title_ ;
};



#endif // GUI_HPP
