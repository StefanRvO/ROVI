#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "ui_SamplePlugin.h"

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <vector>
#include <string>
#include <fstream>

#include "Vision.hpp"
#include "Visualservoing.hpp"


using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rw::math;

using namespace rws;

using namespace cv;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	SamplePlugin();
	virtual ~SamplePlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();

private slots:
	void btnPressed();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);

private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    std::vector<Transform3D<double> > readMotionFile(std::string fileName);
	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rw::models::Device::Ptr device;
    State state;
    std::vector<Transform3D<double> > markerMotions;
    unsigned int counter = 0;
    VisualServoing visualservoing;
    Vision vision;
    std::vector<Vector2D<double> > get_tracker_points(double z, double f, Frame *marker, Frame *camera, int cnt);
    cv::Mat getCameraImage();
    void setCameraViewImage(cv::Mat image);
    std::vector<Vector2D<double> > target;
    std::vector<Vector2D<double> > getVisionPoints(cv::Mat image);
    void keep_velocity_limits(Q &dq);

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
