#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "world.hpp"
#include "player.hpp"

#include <QTimer>
#include <QThread>
#include <QTcpSocket>

#include <iostream>

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
    GUI(Player *p);

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;

    void onUpdate(float delta) override;

    void handleRequest(QTcpSocket *, const QJsonObject &);
    void writeResponse(QTcpSocket *, const QJsonObject &) ;
    QJsonObject stateToJson(const State &state) ;

private slots:
    void newConnection() ;
    void readRequest() ;
    void onSocketStateChanged(QAbstractSocket::SocketState socketState);
    void sendStepResponse(const State &state, bool done) ;

private:

    QTcpServer *server_ ;
    Player *player_ ;
    QMap<QTcpSocket *, QByteArray> connections_ ;
};



#endif // GUI_HPP
