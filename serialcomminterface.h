#ifndef SERIALCOMMINTERFACE_H
#define SERIALCOMMINTERFACE_H

#include <QApplication>
#include <QSerialPort>
#include <QByteArray>
#include <QSerialPortInfo>

typedef unsigned char BYTE;

typedef struct
{
    BYTE S1, S2, S3, S4, S5, S6;
}Servo;

typedef struct
{
    double S1, S2, S3, S4, S5, S6;
}ServoR;


class SerialCommInterface
{
public:
    SerialCommInterface(QString _sPort);
    ~SerialCommInterface();
    bool Send(Servo);
    bool Enabled;
private:
    BYTE Buffer_O[18];
};



#endif // SERIALCOMMINTERFACE_H
