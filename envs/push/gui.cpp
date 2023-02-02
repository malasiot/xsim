#include "gui.hpp"
#include "robot.hpp"
#include "mainwindow.hpp"
#include "world.hpp"
#include "planning_interface.hpp"

#include <iostream>

#include <QTimer>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTcpSocket>
#include <QTcpServer>
#include <QGuiApplication>
#include <QWindow>
#include <QScreen>
#include <QStandardPaths>
#include <QDir>
#include <QPainter>

#include <xviz/scene/node_helpers.hpp>

using namespace std ;
using namespace xviz ;
using namespace xsim ;
using namespace Eigen ;

GUI::GUI(Player *player): SimulationGui(player->world()), player_(player) {
    server_ = new QTcpServer(this) ;
    if ( !server_->listen(QHostAddress(QHostAddress::LocalHost), 7000)) {
        qDebug() << QString("Unable to start the server: %1.").arg(server_->errorString());
        server_->close();
    }

    connect(server_, &QTcpServer::newConnection, this, &GUI::newConnection);

    initCamera({0, 0, 0}, 0.5, SceneViewer::ZAxis) ;

    qRegisterMetaType<State>();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &GUI::grabScreen);
    count_ = 0;
    title_ = MainWindow::instance()->windowTitle() ;

}

void GUI::setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius)
{
    auto target = physics_->findRigidBody(box) ;
    NodePtr tnode = target->visual() ;

    PhongMaterial *mat = static_cast<PhongMaterial *>(tnode->drawables()[0].material().get()) ;
    mat->setDiffuseColor({0.7, 0.2, 0.2}) ;


    NodePtr circle = NodeHelpers::makeCircle({pos.x(), pos.y(), 0.01}, {0, 0, 1}, radius, {1, 0, 0}) ;

    scene_->addChild(circle) ;

}


void GUI::onUpdate(float delta) {
    SimulationGui::onUpdate(delta) ;

    //     vector<ContactResult> results ;
    //    physics.contactPairTest(target_, table_mb->getLink("baseLink"), 0.01, results) ;
}


void GUI::handleRequest(QTcpSocket *socket, const QJsonObject &req) {
    QString req_key = req["request"].toString() ;

    if ( req_key == "reset" ) {
        player_->reset() ;

        QJsonObject resp ;

        auto state = player_->getState() ;

        resp.insert("state", stateToJson(state)) ;

        QJsonArray feasible ;
        for( int64_t action_idx: player_->getFeasibleActions(state) )
            feasible.append(qint64(action_idx)) ;
        resp.insert("feasible", feasible) ;

        writeResponse(socket, resp) ;
    } else if ( req_key == "step" ) {
        int64_t action_id = req["action"].toInt() ;
        ExecuteStepThread *workerThread = new ExecuteStepThread(player_, action_id) ;
        connect(workerThread, &ExecuteStepThread::updateScene, this, [this]() { update();});
        connect(workerThread, &ExecuteStepThread::finished, workerThread, &ExecuteStepThread::deleteLater);
        connect(workerThread, &ExecuteStepThread::finishedStep, this,
                [this, socket](const State &state, bool done) {
            QJsonObject o = stateToJson(state);
            QJsonObject response ;
            response.insert("state", o) ;
            response.insert("done", done) ;

            QJsonArray feasible ;
            for( int64_t action_idx: player_->getFeasibleActions(state) )
                feasible.append(qint64(action_idx)) ;
            response.insert("feasible", feasible) ;

            writeResponse(socket, response);
        });
        workerThread->start();
    } else if ( req_key == "info" ) {
        QJsonObject info ;
        info.insert("boxes", (int)player_->numBoxes()) ;
        info.insert("actions", (int)player_->numActions()) ;
        writeResponse(socket, info);
    }

}

void GUI::readRequest() {
    QTcpSocket* socket = static_cast<QTcpSocket*>(QObject::sender());
    QByteArray &buffer = connections_[socket] ;

    QByteArray data = socket->readAll() ;

    if ( !data.isEmpty() )
        buffer.append(data) ;

    if ( !buffer.endsWith(';') ) return ;

    QJsonDocument doc = QJsonDocument::fromJson(buffer.chopped(1));

    QJsonObject req ;
    if ( !doc.isNull()  && doc.isObject() ) {
        req = doc.object() ;
    } else {

        qDebug() << "Invalid JSON...\n" << data << endl;
    }

    if ( !req.isEmpty() ) {
        handleRequest(socket, req) ;
    }
}

void GUI::writeResponse(QTcpSocket *socket, const QJsonObject &resp) {

    socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact).append(";"));

    socket->flush();

    socket->waitForBytesWritten(3000);

    socket->close();
}

QJsonObject GUI::stateToJson(const State &state) {
    QJsonObject o ;
    o.insert("type", state.toString().c_str()) ;

    QJsonArray boxes ;
    for( const auto &bs: state.boxes_ ) {
        QJsonObject so ;
        so.insert("x", bs.cx_) ;
        so.insert("y", bs.cy_) ;
        so.insert("theta", bs.theta_) ;
        so.insert("name", bs.name_.c_str()) ;
        boxes.append(so) ;
    }

    o.insert("boxes", boxes) ;
    return o ;
}

void GUI::onSocketStateChanged(QAbstractSocket::SocketState socketState) {
    if (socketState == QAbstractSocket::UnconnectedState)  {
        QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
        connections_.remove(sender);
    }
}

void GUI::startRecording() {
    stopRecording() ;
    count_ = 0 ;
    timer_->start(200) ;
    MainWindow::instance()->setWindowTitle(title_ + " (recording)");

}


void GUI::grabScreen() {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (const QWindow *window = windowHandle())
        screen = window->screen();
    if (!screen)
        return;

    auto image = grabFramebuffer();

    if (grab_frame_path_.isEmpty()) {
        grab_frame_path_ = QDir::currentPath();
        grab_frame_path_ += "/grab" ;
    }

    QString file_name = QString("%1_%2.png").arg(grab_frame_path_).arg((int)count_, 4, 10, QLatin1Char('0'));


    if (!image.save(file_name)) {
        qDebug() << "The image could not be saved to " << QDir::toNativeSeparators(file_name);
    }

    count_++;
}

void GUI::stopRecording() {
    timer_->stop();
    MainWindow::instance()->setWindowTitle(title_);
}


void GUI::newConnection() {
    QTcpSocket *socket = server_->nextPendingConnection() ;
    connect(socket, SIGNAL(readyRead()), this, SLOT(readRequest()));
    connect(socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(onSocketStateChanged(QAbstractSocket::SocketState)));
    connections_.insert(socket, QByteArray());
    connect(socket, &QAbstractSocket::disconnected, socket, &QObject::deleteLater);
}

void GUI::keyPressEvent(QKeyEvent *event) {
    int key = event->key() ;
    if ( key == Qt::Key_R ) {
        startRecording() ;
    } else if ( key == Qt::Key_S ) {
        stopRecording() ;
    } else
        SceneViewer::keyPressEvent(event);
}

