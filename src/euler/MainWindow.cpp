#include "MainWindow.h"
#include "RotationControl.h"
#include "Interpolation.h"

#include <QVBoxLayout>

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/console.h>


MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent), spinner(1)
{
	server.reset(new interactive_markers::InteractiveMarkerServer("euler","",false));
	setupUi();

	server->applyChanges();
	spinner.start();

	timer = new QTimer(this);
	this->t = 0.0;
	
}

MainWindow::~MainWindow()
{
	spinner.stop();
	timer->stop();
}

static void linkAxes(RotationControl *f1, RotationControl *f2) {
	QObject::connect(f1, SIGNAL(axesChanged(uint,uint,uint)),
	                 f2, SLOT(setEulerAxes(uint,uint,uint)));
	QObject::connect(f2, SIGNAL(axesChanged(uint,uint,uint)),
	                 f1, SLOT(setEulerAxes(uint,uint,uint)));
}



void MainWindow::setupUi() {
	QWidget *central = new QWidget(this);

	double s = 0.5;
	QColor grey("grey"), red("red"), green("green");
	frame1 = new RotationControl("frame 1", Eigen::Vector3d(-s,s,0), grey, server, this);
	frame11 = new RotationControl("frame 1 - different axis", Eigen::Vector3d(-s,s,5), grey, server, this);
	frame2 = new RotationControl("frame 2", Eigen::Vector3d( s,s,0), grey, server, this);
	frame1p2 = new RotationControl("frame 1+2", Eigen::Vector3d(-s,-s,0), red, server, this);
	frame1c2 = new RotationControl("frame 1*2", Eigen::Vector3d( s,-s,0), green, server, this);
	frameS= new RotationControl("frame slerp", Eigen::Vector3d( 0,3*s,0), green, server, this);

	slide = new QSlider(Qt::Horizontal, this);
	slide->setMinimum(0);
	slide->setMaximum(1000);

	QObject::connect(frame1, SIGNAL(valueChanged(Eigen::Quaterniond)),
	                 frame11, SLOT(setValue(Eigen::Quaterniond)));

	QObject::connect(slide, &QSlider::valueChanged, this, &MainWindow::slerp);

	QVBoxLayout *layout = new QVBoxLayout(central);
	layout->addWidget(frame1);
	layout->addWidget(frame11);
	layout->addWidget(frame2);
	layout->addWidget(frame1p2);
	layout->addWidget(frame1c2);
	layout->addWidget(frameS);
	layout->addWidget(slide);
	this->setCentralWidget(central);

	linkAxes(frame1, frame2);
	linkAxes(frame1, frame1p2);
	linkAxes(frame1, frame1c2);

	frame1p2->setDisabled(true);
	frame1c2->setDisabled(true);
	connect(frame1, SIGNAL(valueChanged(Eigen::Quaterniond)), this, SLOT(updateFrames()));
	connect(frame2, SIGNAL(valueChanged(Eigen::Quaterniond)), this, SLOT(updateFrames()));

	RotationControl *frameI = new RotationControl("interpolated", Eigen::Vector3d(0,3*s,0), QColor("yellow"), server, this);
	layout->addWidget(frameI); frameI->setDisabled(true);
	Interpolation *timer = new Interpolation(frame1->value(), frame2->value(), this);
	connect(frame1, SIGNAL(valueChanged(Eigen::Quaterniond)), timer, SLOT(setStart(Eigen::Quaterniond)));
	connect(frame2, SIGNAL(valueChanged(Eigen::Quaterniond)), timer, SLOT(setEnd(Eigen::Quaterniond)));
	connect(timer, SIGNAL(valueChanged(Eigen::Quaterniond)), frameI, SLOT(setValue(Eigen::Quaterniond)));
	timer->start(100);
}

void MainWindow::updateFrames()
{
	Eigen::Quaterniond q = frame2->value() * frame1->value();
	frame1c2->setValue(q);

	Eigen::Vector3d e = frame1->eulerAngles() + frame2->eulerAngles();
	frame1p2->setEulerAngles(e[0], e[1], e[2]);
}

void MainWindow::rotChanged(){
	Eigen::Vector3d aE = this->frame1->eulerAngles() + this->frame2->eulerAngles();
	Eigen::Quaterniond mQ = this->frame1->value() * this->frame2->value();
	this->frame1p2->setEulerAngles(aE[0],aE[1],aE[2]);
	this->frame1c2->setValue(mQ);
}

void MainWindow::slerp(int i){
	double t = i/1000.0;
	
	Eigen::Quaterniond q1 = this->frame1->value();
	Eigen::Quaterniond q2 = this->frame2->value();

	double  theta = acos(q1.dot(q2));


	if (abs(theta) == 0 )
	{
		this->frameS->setValue(q1);
		return;
	}

	double sT = sin(theta); 
	double ratio1 = sin( (1 - t) *theta )/sT;
	double ratio2 = sin( t*theta )/sT;

	Eigen::Quaterniond slerp = Eigen::Quaterniond::Identity();
	
	slerp.w() = ratio1* q1.w() + ratio2*q2.w();
	slerp.x() = ratio1* q1.x() + ratio2*q2.x();
	slerp.y() = ratio1* q1.y() + ratio2*q2.y();
	slerp.z() = ratio1* q1.z() + ratio2*q2.z();

	this->frameS->setValue(slerp);
}