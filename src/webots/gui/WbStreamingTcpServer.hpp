#ifndef WB_STREAMING_TCP_SERVER_HPP
#define WB_STREAMING_TCP_SERVER_HPP

#include <QtNetwork/QSslConfiguration>
#include <QtNetwork/QTcpServer>

class QSslSocket;

class WbStreamingTcpServer : public QTcpServer {
  Q_OBJECT
public:
  explicit WbStreamingTcpServer(QObject *parent = NULL) : QTcpServer(parent), mSsl(false){};

  void setSslConfiguration(QSslConfiguration &configuration);

protected:
  void incomingConnection(qintptr socketDescriptor) override;

private:
  bool mSsl;
  QSslConfiguration mSslConfiguration;
};

#endif
