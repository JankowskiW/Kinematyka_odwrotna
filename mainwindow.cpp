#include "mainwindow.h"
#include "ui_mainwindow.h"

int Xpp = 20, Ypp = 0, Zpp = 10, Ppp = 0;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //serial = new QSerialPort(this);


    //comm = new CommInterface();

    gflag = 0;
    Runf = 0;
    StepCount = 100;
    ProgramCounter = 0;
    scnt = 0;
    pcnt = 0;
    gcnt = 0;

    Pos.S1 = 128;
    Pos.S2 = 128;
    Pos.S3 = 128;
    Pos.S4 = 128;
    Pos.S5 = 128;
    Pos.S6 = 128;

    Pout.S1 = 128;
    Pout.S2 = 128;
    Pout.S3 = 128;
    Pout.S4 = 128;
    Pout.S5 = 128;
    Pout.S6 = 128;

    Program[0] = Pout;
    Srv[0] = Pout;

    th = new MThread();
    connect(th,SIGNAL(tick()),this,SLOT(ex_th_tick()));
    th->start(th->HighestPriority);
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts())
    {
        ui->cbbSerialPort->addItem(port.portName());
    }

    gflag = 1;


}

MainWindow::~MainWindow()
{
    th->terminate();
    delete comm;
    delete ui;
}

void MainWindow::ex_th_tick()
{
    if(Runf==1)
    {
//        ui->lblProgramStep->setText(QString("Step: %1").arg(pcnt+1));

        if(scnt==0)
        {
            Pos.S1=(double)Pout.S1;
            dx.S1 = (Program[pcnt].S1-Pout.S1)/(double)StepCount;
            Pos.S2=(double)Pout.S2;
            dx.S2 = (Program[pcnt].S2-Pout.S2)/(double)StepCount;
            Pos.S3=(double)Pout.S3;
            dx.S3 = (Program[pcnt].S3-Pout.S3)/(double)StepCount;
            Pos.S4=(double)Pout.S4;
            dx.S4 = (Program[pcnt].S4-Pout.S4)/(double)StepCount;
            Pos.S5=(double)Pout.S5;
            dx.S5 = (Program[pcnt].S5-Pout.S5)/(double)StepCount;
            Pos.S6=(double)Pout.S6;
            dx.S6 = (Program[pcnt].S6-Pout.S6)/(double)StepCount;
        }

        Pos.S1+=dx.S1;
        Pout.S1=(BYTE)Pos.S1;
        Pos.S2+=dx.S2;
        Pout.S2=(BYTE)Pos.S2;
        Pos.S3+=dx.S3;
        Pout.S3=(BYTE)Pos.S3;
        Pos.S4+=dx.S4;
        Pout.S4=(BYTE)Pos.S4;
        Pos.S5+=dx.S5;
        Pout.S5=(BYTE)Pos.S5;
        Pos.S6+=dx.S6;
        Pout.S6=(BYTE)Pos.S6;

        comm->Send(Pout);

        ui->servo_1->setValue(Pout.S1);
        ui->servo_2->setValue(Pout.S2);
        ui->servo_3->setValue(Pout.S3);
        ui->servo_4->setValue(Pout.S4);
        ui->servo_5->setValue(Pout.S5);
        ui->servo_6->setValue(Pout.S6);
         scnt++;
         if(scnt>=StepCount)
         {
             scnt=0;
             if(pcnt<ProgramCounter)
                 pcnt++;
         }
         if(pcnt>=ProgramCounter)
         {
             Runf=0;
             ui->actionRun_Program->setEnabled(true);
         }
         Srv[gcnt]=Pout;
         gcnt++;
         ui->centralWidget->repaint();
         if(gcnt > StepCount*10)
             gcnt=0;
    }
    static int counter=0;
   // ui->lblProgramStep->setText(QString().setNum(counter++));
}

void MainWindow::paintEvent(QPaintEvent *)
{
    int sx = centralWidget()->x()+MX; // przesunięcie całej siatki w poziomie   275+40 = 315
    int sy = centralWidget()->y()+MY; // przesunięcie całej siatki w pionie     275+20 = 90
    int ex = centralWidget()->width()+sx-2*MX; // zwężenie całej siatki w poziomie
    int ey = centralWidget()->width()+sy-2*MY; // zwężenie całej siatki w pionie

    double dx=(ex-sx)/(double)TX;
    double dy=(ey-sy)/(double)TY;

    QPainter painter(this);
    QPen pen;

    pen.setWidth(1);
    painter.setRenderHint(QPainter::Antialiasing,true);

    pen.setColor(QColor(255,255,255,255));
    painter.setPen(pen);
    painter.setBrush(QColor(255,255,255,255));
    painter.drawRect(centralWidget()->x(),centralWidget()->y(),
                     centralWidget()->width(), centralWidget()->height());

    pen.setStyle(Qt::DashLine);
    pen.setColor(QColor(192,192,192,255));
    painter.setPen(pen);

    QFont font;

    font.setPointSize(8);
    painter.setFont(font);

    // ##################### SIATKA #############################################
    for(int x=0; x<=TX; x++)
        painter.drawLine(QLineF(sx+x*dx, sy, sx+x*dx, ey));
    for(int y=0; y<=TY; y++)
        painter.drawLine(QLineF(sx, sy+y*dy, ex, sy+y*dy));

    // ##################### OPIS ###############################################
    pen.setColor(QColor(0,0,0,255));
    painter.setPen(pen);
    for(int x=0; x<=TX; x++)
        painter.drawText(QPoint(sx-4+x*dx, ey+font.pointSize()),
                         QString("%1").arg(x));
    for(int y=0; y<=TY; y++)
        painter.drawText(QPoint(sx-20, sy+(font.pointSize()/2)+y*dy),
                         QString("%1").arg(256-(256*y/TY)));

    double lx=(ex-sx)/1000.0; double ly=(ey-sy)/256.0;
    if(gflag) {
        pen.setStyle(Qt::SolidLine);
        pen.setWidth(2);
        pen.setColor(QColor( 255,0,0,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S1*ly, sx+i*lx, ey-Srv[i].S1*ly));
        pen.setColor(QColor( 255,255,0,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S2*ly, sx+i*lx, ey-Srv[i].S2*ly));

        pen.setColor(QColor( 0,255,0,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S3*ly, sx+i*lx, ey-Srv[i].S3*ly));

        pen.setColor(QColor( 0,255,255,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S4*ly, sx+i*lx, ey-Srv[i].S4*ly));

        pen.setColor(QColor( 255,128,0,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S5*ly, sx+i*lx, ey-Srv[i].S5*ly));

        pen.setColor(QColor( 0,0,255,255 ));
        painter.setPen(pen);
        for ( int i=1; i<gcnt; i++ )
            painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S6*ly, sx+i*lx, ey-Srv[i].S6*ly));
    }
}


void MainWindow::on_servo_1_valueChanged(int value)
{
    ui->lblServo_1->setText(QString().setNum(value));
    Pout.S1=value;
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_servo_2_valueChanged(int value)
{
    ui->lblServo_2->setText(QString().setNum(value));
    Pout.S2=value;
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_servo_3_valueChanged(int value)
{
    ui->lblServo_3->setText(QString().setNum(value));
    Pout.S3=value;
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_servo_4_valueChanged(int value)
{
    ui->lblServo_4->setText(QString().setNum(value));
    Pout.S4=value;
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_servo_5_valueChanged(int value)
{
    ui->lblServo_5->setText(QString().setNum(value));
    Pout.S5=value;
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_servo_6_valueChanged(int value)
{
    ui->lblServo_6->setText(QString().setNum(value));
    Pout.S6=value;
    if(Runf==0)
         comm->Send(Pout);
}


void MainWindow::on_actionNew_triggered()
{
    ui->actionRun_Program->setEnabled(false);
    ui->textBrowser->clear();
    gcnt=0;
    ProgramCounter=0;
    pcnt=0;
    ui->lblProgramStep->setText(QString("Step: %1").arg(ProgramCounter+1));
    ui->servo_1->setValue(128);
    ui->servo_2->setValue(128);
    ui->servo_3->setValue(128);
    ui->servo_4->setValue(128);
    ui->servo_5->setValue(128);
    ui->servo_6->setValue(128);
    ui->dwServoControl->setEnabled(true);
    ui->actionAdd_Step->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
    ui->actionRun_Program->setEnabled(false);
    Pout.S1=128;
    Pout.S2=128;
    Pout.S3=128;
    Pout.S4=128;
    Pout.S5=128;
    Pout.S6=128;
    Program[0]=Pout;
    Srv[0]=Pout;


}

void MainWindow::on_actionAdd_Step_triggered()
{
       Program[ProgramCounter]=Pout;
       ui->lblProgramStep->setText(QString("Step: %1").arg(ProgramCounter+1));
       ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n", ProgramCounter+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));
       ui->actionEnd_Step->setEnabled(true);
       ui->actionRun_Program->setEnabled(false);
       if(ProgramCounter<MAXSTEP)
           ProgramCounter++;
}

void MainWindow::on_actionEnd_Step_triggered()
{
    ui->actionRun_Program->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
}

void MainWindow::on_actionRun_Program_triggered()
{
    scnt = 0;
    pcnt = 0;
    gcnt = 0;
    Runf = 1;
    ui->actionRun_Program->setEnabled(false);
    //kinematics();
}

void MainWindow::on_actionSave_triggered()
{
    QString FileName;
    FileName = QFileDialog::getSaveFileName(this, tr("Open File"),
    "/sekwencja",tr("Robot Files (*.txt *.rob)"));

    QFile file(FileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

    file.write((char*)Program,sizeof(Servo)*ProgramCounter);
    file.close();
}

void MainWindow::on_actionOpen_triggered()
{
    on_actionNew_triggered();

        QString FileName;
        FileName = QFileDialog::getOpenFileName(this, tr("Open File"),
        "/home/robot", tr("Robot Files (*.txt *.rob)"));

        QFile file(FileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            return;

        QByteArray readedbytes = file.readAll();
        file.close();

        int i;
        int workers = readedbytes.size()/sizeof(Servo);
        for(i = 0; i<workers; i++){
            Pout.S1 = (int)readedbytes.at(sizeof(Servo)*i);
            Pout.S2 = (int)readedbytes.at(sizeof(Servo)*i+1);
            Pout.S3 = (int)readedbytes.at(sizeof(Servo)*i+2);
            Pout.S4 = (int)readedbytes.at(sizeof(Servo)*i+3);
            Pout.S5 = (int)readedbytes.at(sizeof(Servo)*i+4);
            Pout.S6 = (int)readedbytes.at(sizeof(Servo)*i+5);
            Program[ProgramCounter]=Pout;
            ui->lblProgramStep->setText(QString("Step: %1").arg(ProgramCounter+1));
            ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
            ProgramCounter+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));
            ProgramCounter++;
        }

        ui->actionRun_Program->setEnabled(true);
        ui->actionEnd_Step->setEnabled(false);
}

void MainWindow::on_cbbMode_activated(int index)
{
    if (index == 0)
    {
        ui->dwControlPanel->setEnabled(false);
        ui->dwServoControl->setEnabled(true);
    }else
    {
        ui->dwControlPanel->setEnabled(true);
        ui->dwServoControl->setEnabled(false);
    }
}

void MainWindow::kinematics(int LastChanged)
{
    char X,Y,Z,P, Xp;
    double Xb, Zb, p1, p2, t1, t2, t3, Q,
           Ph0, Ph1, Ph2, Ph3, Ph0st, Ph1st, Ph2st, Ph3st,
           Svo1, Svo2, Svo3, Svo4,Svo5, Svo6,
           V1[2] = {0}, V2[2] = {0}, V3[3] = {0},
           Gx[4] = {0}, Gy[4] = {0}, Gz[4] = {0},
           R0[4][4] = {0}, R1[4][4] = {0}, R2[4][4] = {0}, R3[4][4] = {0},
           T0[4][4] = {0}, T1[4][4] = {0}, T2[4][4] = {0}, T3[4][4] = {0},
           A0[4][4] = {0}, A1[4][4] = {0}, A2[4][4] = {0}, A3[4][4] = {0},
           P0[4][4] = {0}, P1[4][4] = {0}, P2[4][4] = {0}, P3[4][4] = {0};

    /* MESSAGE BOX TO INFORM ABOUT OUT OF RANGE */
    QMessageBox msgBox;
    msgBox.setText("Servo|Cooridinates out of range.");
    msgBox.setInformativeText("Press OK to continue.");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    /* ----------------------------------------- */

    X = ui->spbX->value();  // Wspolrzedna X
    Y = ui->spbY->value();  // Wspolrzedna Y
    Z = ui->spbZ->value();  // Wspolrzedna Z
    P = ui->spbP->value();  // Kat nachylenia do obiektu
    qDebug("\nX = %d,\nY = %d,\n Z = %d,\n P = %d",X,Y,Z,P);
    Xp = X;
    X = qSqrt(X*X + Y*Y);

  //  ui->label_3->setText(QString().sprintf("X = %d",X));
  //  ui->label_4->setText(QString().sprintf("Y = %d",Y));
   // ui->label_5->setText(QString().sprintf("Z = %d",Z));
   // ui->label->setText(QString().sprintf("%f",qCos(qAtan2(ui->spbY->value(),X) * 180 / M_PI)));

    // -------------------- KINEMATYKA ODWROTNA --------------------------
    Xb = (X - L3 * qCos(P * M_PI / 180)) / (2 * L1);
    Zb = (Z - H - L3 * qSin(P * M_PI / 180)) / (2 * L1);

    Q = qSqrt((1 / (Xb * Xb + Zb * Zb)) - 1);

    p1 = qAtan2(Zb + Q * Xb, Xb - Q * Zb) * (180 / M_PI);
    p2 = qAtan2(Zb - Q * Xb, Xb + Q * Zb) * (180 / M_PI);
   // ui->label_6->setText(QString().sprintf("%f, %f, %f, %f",R0[3][0],R0[3][1],R0[3][2],R0[3][3]));
    t1 = p1-90; // stopnie
    t2 = p2-t1; // stopnie
    t3 = P-p2;  // stopnie

    // ALBO Ph0 = qAtan2(Y,X);
    // ale wtedy X = qSqrt(X*X + Y*Y);
    Ph0 = qAtan2(Y,Xp);  // radiany
    Ph1 = ((t1 + 90) * 2 * M_PI) / 360;  // radiany
    Ph2 = ((t2 - 90) * 2 * M_PI) / 360;  // radiany
    Ph3 = (t3 * 2 * M_PI) / 360;  // radiany
    Ph0st = Ph0*180/M_PI;
    if(Ph0st<-(SV0/2)) Ph0st = -(SV0/2);
    if(Ph0st>(SV0/2)) Ph0st = (SV0/2);
    Ph1st = Ph1*180/M_PI;
    if(Ph1st<-(SV1/2)) Ph1st = -(SV1/2);
    if(Ph1st>(SV1/2)) Ph1st = (SV1/2);
    Ph2st = Ph2*180/M_PI;
    if(Ph2st<-(SV2/2)) Ph2st = -(SV3/2);
    if(Ph2st>(SV2/2)) Ph2st = (SV3/2);
    Ph3st = Ph3*180/M_PI;
    if(Ph3st<-(SV3/2)) Ph3st = -(SV3/2);
    if(Ph3st>(SV3/2)) Ph3st = (SV3/2);
    ui->label_3->setText(QString().sprintf("0 = %f",Ph0st));
    ui->label_4->setText(QString().sprintf("1 = %f",Ph1st));
    ui->label_5->setText(QString().sprintf("2 = %f",Ph2st));
    ui->label_6->setText(QString().sprintf("3 = %f",Ph3st));

    if(qIsNaN(Ph1)||qIsNaN(Ph2)||qIsNaN(Ph3))
    {
        if(LastChanged == 1)
        {
            ui->spbX->setValue(Xpp);
        }else if(LastChanged == 2)
        {
            ui->spbY->setValue(Ypp);
        }else if(LastChanged == 3)
        {
            ui->spbZ->setValue(Zpp);
        }else
        {
            ui->spbP->setValue(Ppp);
        }
        msgBox.exec();
        return;
    }

    Xpp = X;
    Ypp = Y;
    Zpp = Z;
    Ppp = P;
//    ui->test_2->setText(QString::number(Ph1));
//    ui->test_3->setText(QString::number(Ph2));
//    ui->test_4->setText(QString::number(Ph3));

    // --------------------- KINEMATYKA PROSTA ---------------------------
    R0[0][0] = qCos(Ph0);
    R0[0][2] = -qSin(Ph0);
    R0[1][1] = 1;
    R0[2][0] = -qSin(Ph0);
    R0[2][2] = qCos(Ph0);
    R0[3][3] = 1;
    ui->label_2->setText(QString().sprintf("%f",qCos(qAtan2(-20,X) * 180 / M_PI)));
    R1[0][0] = qCos(Ph1);
    R1[0][1] = -qSin(Ph1);
    R1[1][0] = qSin(Ph1);
    R1[1][1] = qCos(Ph1);
    R1[2][2] = 1;
    R1[3][3] = 1;

    R2[0][0] = qCos(Ph2);
    R2[0][1] = -qSin(Ph2);
    R2[1][0] = qSin(Ph2);
    R2[1][1] = qCos(Ph2);
    R2[2][2] = 1;
    R2[3][3] = 1;

    R3[0][0] = qCos(Ph3);
    R3[0][1] = -qSin(Ph3);
    R3[1][0] = qSin(Ph3);
    R3[1][1] = qCos(Ph3);
    R3[2][2] = 1;
    R3[3][3] = 1;

    T0[0][0] = 1;
    T0[1][1] = 1;
    T0[1][3] = H;
    T0[2][2] = 1;
    T0[3][3] = 1;

    T1[0][0] = 1;
    T1[1][1] = 1;
    T1[0][3] = L1;
    T1[2][2] = 1;
    T1[3][3] = 1;

    T2[0][0] = 1;
    T2[1][1] = 1;
    T2[0][3] = L2;
    T2[2][2] = 1;
    T2[3][3] = 1;

    T3[0][0] = 1;
    T3[1][1] = 1;
    T3[0][3] = L3;
    T3[2][2] = 1;
    T3[3][3] = 1;

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                A0[i][j] += R0[i][k]*T0[k][j];

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                A1[i][j] += R1[i][k]*T1[k][j];

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                A2[i][j] += R2[i][k]*T2[k][j];

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                A3[i][j] += R3[i][k]*T3[k][j];



    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            P0[i][j] = A0[i][j];

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                P1[i][j] += A0[i][k]*A1[k][j];

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                P2[i][j] += P1[i][k]*A2[k][j];
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                P3[i][j] += P2[i][k]*A3[k][j];


/* =========================================================== */
/* ===         USTALANIE WARTOSCI SERWOMECHANIZMOW         === */
/* =========================================================== */
    Svo6 = 128 - (Ph0*180/M_PI)*(255/SV3); // PIERWSZE OD DOLU (OBROT)
    if (Svo6 > 255) Svo6 = 255;
    if (Svo6 < 0) Svo6 = 0;
    Svo5 = 128 - (Ph1*180/M_PI-90)*(255/SV2); // DRUGIE OD DOLU LUB Svo5 = 128 - (Ph1*180/M_PI - 90)*(255/SV2)
    if (Svo5 > 255) Svo5 = 255;
    if (Svo5 < 0) Svo5 = 0;
    Svo4 = 128 + (Ph2*180/M_PI)*(255/SV1); // TRZECIE OD DOLU
    if (Svo4 > 255) Svo4 = 255;
    if (Svo4 < 0) Svo4 = 0;
    Svo3 = 128 - (Ph3*180/M_PI-75)*(255/SV0); // CZWARTE OD DOLU
    if (Svo3 > 255) Svo3 = 255;
    if (Svo3 < 0) Svo3 = 0;
    Svo2 = 128;
    Svo1 = 128;
  //  ui->label_2->setText(QString().sprintf("%f",Ph3));
    Pout.S6 = (int)Svo6;
    Pout.S5 = (int)Svo5;
    Pout.S4 = (int)Svo4;
    Pout.S3 = (int)Svo3;
    Pout.S2 = 128;
    Pout.S1 = 128;
    Program[ProgramCounter]=Pout;
    qDebug("\nP3 = %f,\nP4 = %f,\nP5 = %f,\nP6 = %f",Ph1,Ph2,Ph3,Ph0);
    qDebug("\n\nS3 = %d,\nS4 = %d,\nS5 = %d,\nS6 = %d",Pout.S3,Pout.S4,Pout.S5,Pout.S6);
    qDebug("\n\nS3 = %d,\nS4 = %d,\nS5 = %d,\nS6 = %d",Pout.S3,Pout.S4,Pout.S5,Pout.S6);

/* =========================================================== */
/* ===                        PLOTING                      === */
/* =========================================================== */
    V1[0] = P1[0][3];
    V1[1] = P1[1][3];
    V2[0] = P2[0][3];
    V2[1] = P2[1][3];
    V3[0] = P3[0][3];
    V3[1] = P3[1][3];

    Gx[0] = 0;
    Gx[1] = V1[0];
    Gx[2] = V2[0];
    Gx[3] = V3[0];

    Gy[0] = H;
    Gy[1] = V1[1];
    Gy[2] = V2[1];
    Gy[3] = V3[1];

    Gz[0] = 0;
    Gz[1] = 0;
    Gz[2] = 0;
    Gz[3] = 0;

    MainWindow::cPlot(Gx,Gy);
    MainWindow::aPlot();
}

void MainWindow::on_btnStart_clicked()
{
    kinematics(XX);
    ui->lblProgramStep->setText(QString("Step: %1").arg(ProgramCounter+1));
    ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
             ProgramCounter+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));

    ProgramCounter++;
}

void MainWindow::cPlot(double *X, double *Y)
{
    // generate some data:
    QVector<double> x(4),y(4);


    for(int i=0; i<4; i++)
    {
        x[i] = X[i];
        y[i] = Y[i];
    }
    ui->plotFront->addGraph();
    ui->plotFront->graph(0)->setData(x,y,TRUE);
    ui->plotFront->xAxis->setLabel("x");
    ui->plotFront->yAxis->setLabel("z");
    ui->plotFront->xAxis->setRange(-40,40);
    ui->plotFront->yAxis->setRange(0, 40);
    ui->plotFront->replot();

}

void MainWindow::aPlot()
{


    QVector<double> x(3),y(3);
    int xx,yy,zz;
    double qq,xp,yp,len;
    xx = ui->spbX->value();
    yy = ui->spbY->value();
    zz = ui->spbZ->value();
    len = qSqrt(qPow(xx,2)+qPow(yy,2));
    qq = qSqrt(qPow(L1,2)-qPow(zz/2,2));
    yp = -(qq*(yy/len));
    xp = -(qq*(xx/len));
    x[0] = xp;
    y[0] = yp;
    x[1] = 0;
    y[1] = 0;
    x[2] = xx;
    y[2] = yy;
    ui->plotAbove->addGraph();
    ui->plotAbove->graph(0)->setData(x,y);
    ui->plotAbove->xAxis->setLabel("x");
    ui->plotAbove->yAxis->setLabel("y");
    ui->plotAbove->xAxis->setRange(-40,40);
    ui->plotAbove->yAxis->setRange(-40, 40);
    ui->plotAbove->replot();
}

void MainWindow::on_spbX_valueChanged(int value)
{
    kinematics(XX);
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_spbY_valueChanged(int value)
{
    kinematics(YY);
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_spbZ_valueChanged(int value)
{
    kinematics(ZZ);
    if(Runf==0)
         comm->Send(Pout);

}

void MainWindow::on_spbP_valueChanged(int value)
{
    kinematics(PP);
    if(Runf==0)
         comm->Send(Pout);
}

void MainWindow::on_cbbSerialPort_currentIndexChanged(const QString &sPort)
{

}

void MainWindow::on_cbbSerialPort_currentTextChanged(const QString &sPort)
{
   //     qDebug("\nport = %s",sPort);
   // SerialCommInterface::SerialCommInterface(sPort);
}

void MainWindow::on_cbbSerialPort_currentIndexChanged(int index)
{
    QString sPort;
    sPort = ui->cbbSerialPort->currentText();

    qDebug("\nport = %s",qUtf8Printable(sPort));
    comm = new SerialCommInterface(sPort);
    //SerialCommInterface::SerialCommInterface(sPort);

}
