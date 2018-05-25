#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
//#include "comminterface.h"
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <string.h>
#include <QtMath>
#include <QtDebug>
#include <QMessageBox>
#include "qcustomplot.h"
#include "serialcomminterface.h"
#include "mthread.h"


#define TRUE 1
#define FALSE 0

#define MX 40
#define MY 20

#define TX 10
#define TY 16

#define MAXPOINTS 2000
#define MAXSTEP 1000

#define H 6.5     // Wysokość ramienia
#define L1 8    // Odległość między S1 -> S2
#define L2 8    // Odległość między S2 -> S3
#define L3 20    // Odległość między S3 -> końćem chwytaka

#define XX 1
#define YY 2
#define ZZ 3
#define PP 4

/* =========== ZAKRESY SERW ROBOTA W STOPNIACH ================== */
#define SV0 (160)     // servo 1 (obrotowa podstawa)
#define SV1 (160)     // servo 2 (pierwsze zgiecie)
#define SV2 (160)     // ...
#define SV3 (160)     // ...
#define SV4 (100)     // servo 5 (obrot chwytaka)
#define SV5 (100)     // servo 6 (rozchylenie chwytaka)


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //CommInterface *comm;
    SerialCommInterface *comm;
    Servo Pin, Pout;
    Servo Srv[MAXPOINTS];
    Servo Program[MAXSTEP];
    ServoR Pos, dx;
    int ProgramCounter;
    int StepCount;
    int gcnt;
    int scnt;
    int pcnt;
    int gflag;
    int Runf;

private slots:
    void on_servo_1_valueChanged(int value);
    void on_servo_2_valueChanged(int value);
    void on_servo_3_valueChanged(int value);
    void on_servo_4_valueChanged(int value);
    void on_servo_5_valueChanged(int value);
    void on_servo_6_valueChanged(int value);
    void ex_th_tick();
    void on_actionNew_triggered();
    void on_actionAdd_Step_triggered();
    void on_actionEnd_Step_triggered();
    void on_actionRun_Program_triggered();
    void on_actionSave_triggered();
    void on_actionOpen_triggered();
    void on_btnStart_clicked();
    void on_cbbMode_activated(int index);
    void on_cbbSerialPort_currentIndexChanged(const QString &sPort);

    void on_spbX_valueChanged(int value);
    void on_spbY_valueChanged(int value);
    void on_spbZ_valueChanged(int value);
    void on_spbP_valueChanged(int value);

    void kinematics(int);
    void cPlot(double *, double*);
    void aPlot();


    void on_cbbSerialPort_currentTextChanged(const QString &arg1);

    void on_cbbSerialPort_currentIndexChanged(int index);

private:
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *);
    MThread *th;
};

#endif // MAINWINDOW_H

