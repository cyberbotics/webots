#include <QtNetwork/QSslSocket>
#include "WbStreamingTcpServer.hpp"

void WbStreamingTcpServer::setSslConfiguration(QSslConfiguration &configuration) {
  mSslConfiguration = configuration;
  mSsl = true;
}

void WbStreamingTcpServer::incomingConnection(qintptr socketDescriptor) {
  if (!mSsl) {
    QTcpServer::incomingConnection(socketDescriptor);
    return;
  }

  QSslSocket *serverSocket = new QSslSocket();
  serverSocket->setSocketDescriptor(socketDescriptor);
  serverSocket->setSslConfiguration(mSslConfiguration);
  serverSocket->startServerEncryption();
  addPendingConnection(serverSocket);
};
