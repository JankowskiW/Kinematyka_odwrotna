#include "serialcomminterface.h"

//QString Port;
QSerialPort serial;
SerialCommInterface::SerialCommInterface(QString _sPort)
{
    qDebug("\nwewnatrz = %s",qUtf8Printable(_sPort));
    QString Porcik;

    serial.setPortName(qUtf8Printable(_sPort));
    serial.setBaudRate(QSerialPort::Baud9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.open(QIODevice::WriteOnly);
}

SerialCommInterface::~SerialCommInterface()
{

    serial.close();
}

bool SerialCommInterface::Send(Servo data)
{
    QByteArray QBABuffer;
    BYTE error;
    Buffer_O[0] = 255;
    Buffer_O[1] = 0;
    Buffer_O[2] = data.S1;

    Buffer_O[3] = 255;
    Buffer_O[4] = 1;
    Buffer_O[5] = data.S2;

    Buffer_O[6] = 255;
    Buffer_O[7] = 2;
    Buffer_O[8] = data.S3;

    Buffer_O[9] = 255;
    Buffer_O[10] = 3;
    Buffer_O[11] = data.S4;

    Buffer_O[12] = 255;
    Buffer_O[13] = 4;
    Buffer_O[14] = data.S5;

    Buffer_O[15] = 255;
    Buffer_O[16] = 5;
    Buffer_O[17] = data.S6;
    QBABuffer = QByteArray((char*)Buffer_O,18);
    error = serial.write(QBABuffer);
    if(error != -1)
    {
        return 1;
    }else
        return 0;
}
