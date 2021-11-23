#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QSlider>
#include <interactive_markers/interactive_marker_server.h>
#include <math.h>  
class RotationControl;

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

signals:


public slots:
	void updateFrames();

private:
	void setupUi();


private:
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	ros::AsyncSpinner spinner;
	float t;
	QTimer *timer;

	RotationControl *frame1;
	RotationControl *frame11;
	RotationControl *frame2;
	RotationControl *frame1p2;
	RotationControl *frame1c2;
	RotationControl *frameS;
	QSlider *slide;
};
