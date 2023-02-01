#include "server.hpp"

#include <QTcpServer>
#include <QMessageBox>
#include <QTcpSocket>
#include <QJsonObject>
#include <QJsonDocument>

SimulationServer::SimulationServer(Player *player): player_(player) {
    if ( !listen(QHostAddress(QHostAddress::LocalHost), 7000)) {
        qDebug() << QString("Unable to start the server: %1.").arg(errorString());
        close();
        return;
    }

    connect(this, &QTcpServer::newConnection, this, &SimulationServer::newConnection);
}

QJsonObject SimulationServer::getResponse(const QJsonObject &req) {
    QString req_key = req["request"].toString() ;

    QJsonObject resp ;
    if ( req_key == "reset" ) {
        resp.insert("status", "reset") ;
    } else if ( req_key == "step" ) {
        resp.insert("status", "step") ;
    }

    return resp ;
}

void SimulationServer::readRequest() {
    QTcpSocket* socket = static_cast<QTcpSocket*>(QObject::sender());
    QByteArray &buffer = connections_[socket] ;

    QByteArray data = socket->readAll() ;

    if ( !data.isEmpty() )
        buffer.append(data) ;

    if ( !buffer.endsWith(';') ) return ;

    QJsonDocument doc = QJsonDocument::fromJson(buffer.chopped(1));

     QJsonObject req, resp ;
    if ( !doc.isNull()  && doc.isObject() ) {
        req = doc.object() ;
    } else {

       qDebug() << "Invalid JSON...\n" << data << endl;
    }

    if ( !req.isEmpty() ) {
        resp = getResponse(req) ;
    }

    socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact).append(";"));

    socket->flush();

    socket->waitForBytesWritten(3000);

    socket->close();

}

void SimulationServer::onSocketStateChanged(QAbstractSocket::SocketState socketState) {
    if (socketState == QAbstractSocket::UnconnectedState)  {
        QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
        connections_.remove(sender);
    }
}


void SimulationServer::newConnection() {
    QTcpSocket *socket = nextPendingConnection() ;
    connect(socket, SIGNAL(readyRead()), this, SLOT(readRequest()));
    connect(socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(onSocketStateChanged(QAbstractSocket::SocketState)));
    connections_.insert(socket, QByteArray());

    connect(socket, &QAbstractSocket::disconnected, socket, &QObject::deleteLater);
}
