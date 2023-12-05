#pragma once

#include <QTcpServer>
#include <deque>

#include "player.hpp"

class SimulationServer: public QTcpServer {
     Q_OBJECT
public:
    SimulationServer(Player *player) ;

    QJsonObject getResponse(const QJsonObject &req);

private slots:
    void newConnection() ;
    void readRequest() ;
    void onSocketStateChanged(QAbstractSocket::SocketState socketState);
private:
    QMap<QTcpSocket *, QByteArray> connections_ ;
    Player *player_ ;

};

