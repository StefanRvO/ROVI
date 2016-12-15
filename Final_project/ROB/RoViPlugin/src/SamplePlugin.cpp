#include "SamplePlugin.hpp"
#include "LineFinding.hpp"


#include <QPushButton>


cv::Mat globalImg;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Downloads/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread("/home/student/Downloads/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}
		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		if (cameraFrame != NULL) {
			if (cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
				std::istringstream iss (camParam, std::istringstream::in);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
				_framegrabber->init(gldrawer);
			}
		}

        // Find and create a pointer to the device
        device = _wc->findDevice("PA10");

        // Get default state
        state = _wc->getDefaultState();
	}
}

void SamplePlugin::close() {
	log().info() << "CLOSE" << "\n";

	// Stop the timer
	_timer->stop();
	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
    markerMotions = readMotionFile("/home/student/Downloads/SamplePluginPA10/motions/MarkerMotionFast.txt");

	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        image = ImageLoader::Factory::load("/home/student/Downloads/SamplePluginPA10/markers/Marker2b.ppm");
        _textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/student/Downloads/SamplePluginPA10/backgrounds/lines1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
        //state = getRobWorkStudio()->getState();
        MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
        // Find the marker frame origin coordinate relative to the camera
        Frame* cameraFrame = _wc->findFrame("Camera");

        Q q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
        device->setQ(q, state);
        counter = 0;
        marker->moveTo(markerMotions[0], state);
        getRobWorkStudio()->setState(state);
        target_from_frame = get_tracker_points(0.5, 823., marker, cameraFrame, 3);
        target = target_from_frame;
        cv::Mat image = getCameraImage();
        //target = getVisionPoints(image);
        getRobWorkStudio()->setState(state);
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
            _timer->start(10); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::print_joint_and_tool_pose()
{
    Frame* cameraFrame = _wc->findFrame("Camera");
    Q qVector = device->getQ(state);
    auto tool_pos = device->baseTframe(cameraFrame, state).P();
    auto tool_rot = RPY<> (device->baseTframe(cameraFrame, state).R());
    //std::cout << qVector << std::endl;
    std::cout << counter - 1 << ", ";
    for(uint8_t i = 0; i < qVector.size(); i++)
    {
        std::cout << qVector[i] << ", ";
    }
    std::cout << ", ";
    for(uint8_t i = 0; i < tool_pos.size(); i++)
    {
        std::cout << tool_pos[i] << ", ";
    }
    std::cout << ", ";
    for(uint8_t i = 0; i < tool_rot.size(); i++)
    {
        std::cout << tool_rot[i] << ", ";
    }
    std::cout << std::endl;
}

void SamplePlugin::print_max_displacement(std::vector<Vector2D<double> > uv)
{
    std::vector<Vector2D<double> > duv_s;
    duv_s.resize(uv.size());
    float error = 0;
    for(size_t i = 0; i < duv_s.size(); i++)
    {
        duv_s[i] = target_from_frame[i] - uv[i];
        error += sqrt(duv_s[i][0] * duv_s[i][0] + duv_s[i][1] * duv_s[i][1]);
    }
    error /= duv_s.size();
    if(error >= max_error) max_error = error;
    if(counter == markerMotions.size())
    {
        std::cout << dt << ", " << max_error << std::endl;
        dt -= 0.05;
    }
}

void SamplePlugin::print_max_displacement_joint_pos_joint_velc(std::vector<Vector2D<double> > uv, Q dq)
{
    auto velocityLimits = device->getVelocityLimits() * dt;

    std::vector<Vector2D<double> > duv_s;
    duv_s.resize(uv.size());
    std::vector<float> error;
    for(size_t i = 0; i < duv_s.size(); i++)
    {
        duv_s[i] = target_from_frame[i] - uv[i];
        error.push_back(sqrt(duv_s[i][0] * duv_s[i][0] + duv_s[i][1] * duv_s[i][1]));
    }
    Q qVector = device->getQ(state);
    //Q q_limits = device->getBounds();
    //std:: cout << q_limits << std::endl;



}

void SamplePlugin::timer() {

    //state = getRobWorkStudio()->getState();

    // Find the marker frame and cast it to a moveable frame
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
    // Find the marker frame origin coordinate relative to the camera
    Frame* cameraFrame = _wc->findFrame("Camera");


    marker->moveTo(markerMotions[counter++], state);

    print_joint_and_tool_pose();
    // Use vision to get marker points
    cv::Mat image = getCameraImage();
    std::vector<Vector2D<double> > uv = getVisionPoints(image);
    auto uv_ = get_tracker_points(0.5, 823., marker, cameraFrame, 3);
    //for(auto &uv_pt : uv_) std::cout << uv_pt << "\t";
    //std::cout << std::endl;
    auto d_j = device->baseJframe(cameraFrame, state);
    auto sj = Jacobian(inverse(device->baseTframe(cameraFrame, state).R()));

    Q dq = visualservoing.calculateDeltaQ(uv_, target, 0.5, 823.0,sj,d_j);
    Q qVector = device->getQ(state);
    qVector += keep_velocity_limits(dq);


    //std::cout << "Qvector after: " << qVector << std::endl;
    device->setQ(qVector, state);
    getRobWorkStudio()->setState(state);
    uv_ = get_tracker_points(0.5, 823., marker, cameraFrame, 3);
    //print_max_displacement(uv_);
    if(counter == markerMotions.size())
    {
        qVector =  Q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
        counter = 0;
        marker->moveTo(markerMotions[0], state);

        device->setQ(qVector, state);
        getRobWorkStudio()->setState(state);
    }
}


/*
*   Get point of the marker from the vision image tracker
*/
std::vector<Vector2D<double> > SamplePlugin::getVisionPoints(cv::Mat image)
{

    // Find the marker frame and cast it to a moveable frame
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
    // Find the marker frame origin coordinate relative to the camera
    Frame* cameraFrame = _wc->findFrame("Camera");

    // Get the tracked point from vision
    cv::Mat fixed_colours;
    cv::cvtColor(image, fixed_colours, CV_BGR2RGB);    // Convert image to HSV

    LineFinding line_f(fixed_colours);

    std::vector<cv::Point2f> trackedPoints = vision.trackPicture(image);
    //std::vector<cv::Point2f> trackedPoints = line_f.get_marker_points(&image);
    if(trackedPoints.size())
    {
        trackedPoints.erase(trackedPoints.begin() + 0);
    //    trackedPoints.erase(trackedPoints.begin() + 1);
    //    trackedPoints.erase(trackedPoints.begin() + 2);
    }
    // Get a image showing the tracked image from vision and set it to camera view
    //cv::Mat trackedImage = vision.getVisionViewImage(image,trackedPoints);

    // Convert from Point2f to Vector2D
    std::vector<Vector2D<double> > convertedPoints;
    for(size_t i = 0; i < trackedPoints.size(); i++)
    {
        auto  &point = trackedPoints[i];

        Vector2D<double> new_point( (point.x - image.cols / 2) , -point.y + image.rows / 2);
        convertedPoints.push_back(new_point);
        //std::cout << new_point << "\t" << image.cols /2 << std::endl;

    }

    setCameraViewImage(image);
    //std::cout << std::endl;
    //if(target.size()) return target;
    return convertedPoints;
}


/*
*   Grabs the camera image and returns it as a opencv matrix
*/
cv::Mat SamplePlugin::getCameraImage()
{
    if (_framegrabber != NULL) {
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        //Mat imflip;
        //cv::flip(im, imflip, 0);

        return im;
	}
    return cv::Mat();

}

/*
*   Sets the camera view label to a given image
*/
void SamplePlugin::setCameraViewImage(cv::Mat image)
{
    // Show in QLabel

    QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
}


Q SamplePlugin::keep_velocity_limits(Q &dq)
{
    //std::cout << "dq\t" << dq << std::endl;
     auto velocityLimits = device->getVelocityLimits() * dt;
     //std::cout << "v_limit\t" << velocityLimits << std::endl;
     for(uint8_t i = 0; i < dq.size(); i++)
     {
         //std::cout << dq[i] << std::endl;
         if(dq[i] > 0)
            dq[i] = min(velocityLimits[i], dq[i]);
        else
            dq[i] = -min(velocityLimits[i], -dq[i]);
         //std::cout << dq[i] << std::endl << std::endl;
     }
     //std::cout << "dq_mod\t" << dq << std::endl << std::endl;

     return dq;

}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

/*
*   Reads a file with marker movements with (x,y,z,r,p,y) on each line
*   and converts each line/move to a transformation matrix
*   Returns a vector of all the transformation matrixes
*/
std::vector<Transform3D<double> > SamplePlugin::readMotionFile(std::string fileName)
{

    std::string line;
    double x, y, z, roll, pitch, yaw;
    RPY<double> rpy;
    Vector3D<double> xyz;
    std::vector<Transform3D<double> > markerMotions; // Vector of transformations (rpy and translation)

    std::ifstream file(fileName);

    if(file.is_open())
    {
        // Read each line of the file
        while(std::getline(file,line))
        {
            std::stringstream lineStream(line); // Create a stream for the line string
            //std::cout << line << std::endl;
            // Read x,y,z,r,p,y from the line string
            lineStream >> x;
            lineStream >> y;
            lineStream >> z;
            lineStream >> roll;
            lineStream >> pitch;
            lineStream >> yaw;

            rpy = RPY<double>(roll,pitch,yaw);   // Create RPY matrix
            xyz = Vector3D<double>(x,y,z);   // Create translation vector

            // Create a transformation matrix from the RPY and translation vector and put it into a vector
            markerMotions.push_back(Transform3D<double>(xyz, rpy.toRotation3D() ));
        }

        file.close();
    }

    return markerMotions;
}

std::vector<Vector2D<double> > SamplePlugin::get_tracker_points(double z, double f, Frame *marker, Frame *camera, int cnt)
{
    std::vector<Vector2D<double>> points;
    Transform3D<> marker_coords = inverse(marker->fTf(camera, state));
    std::vector<Vector3D<> > marker_points;
    //Create
    if(cnt == 1)
        marker_points.push_back(marker_coords * Vector3D<>(0,0,0));
    else
        marker_points.push_back(marker_coords * Vector3D<>(0.125,-0.125,0));
    if(cnt >= 2)
        marker_points.push_back(marker_coords * Vector3D<>(0.125,0.19,0));
    if(cnt >= 3)
        marker_points.push_back(marker_coords * Vector3D<>(-0.175,-0.125,0));
    for(auto marker_point : marker_points)
    {
        Vector2D<double> point;
        point[0] = f * marker_point[0] / z;
        point[1] = f * marker_point[1] / z;
        points.push_back(point);
    }
    return points;
}



Q_EXPORT_PLUGIN(SamplePlugin);
