#include "SamplePlugin.hpp"
#include "LineFinding.hpp"


#include <QPushButton>

#include "Vision.hpp"


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
    /*
    state= getRobWorkStudio()->getState();
    Q qVector = device->getQ(state);
    std::cout << qVector << std::endl;
    Q dq(7,1);
    qVector += dq;
    device->setQ(qVector, state);
    getRobWorkStudio()->setState(state);
    std::cout << qVector << std::endl;

    auto marker = _wc->findFrame("Marker");
    std::vector<Transform3D<float> > test = readMotionFile("/home/student/Dropbox/RobTek/Cand_1_semester/Robotics/Mandatory/ROVI/Final_project/ROB/RoViPlugin/motions/MarkerMotionSlow.txt");
    */

    markerMotions = readMotionFile("/home/student/Downloads/SamplePluginPA10/motions/MarkerMotionSlow.txt");


	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        image = ImageLoader::Factory::load("/home/student/Downloads/SamplePluginPA10/markers/Marker2b.ppm");
        _textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/student/Downloads/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
            _timer->start(1000); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {

    state = getRobWorkStudio()->getState();

    // Find the marker frame and cast it to a moveable frame
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");

    // Find the marker frame origin coordinate relative to the camera
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    Transform3D<double> markerTransformRelativeToCam = cameraFrame->fTf(marker, state);
    Vector3D<double> markerPosRelativeToCam = markerTransformRelativeToCam.P();

    // Set the marker to a new position from the markermotion vector
    //Transform3D<double> tmp = markerMotions[counter];
    //std::cout << counter << std::endl;
    //std::cout << markerMotions.size() << std::endl;
    marker->setTransform(markerMotions[counter], state);

    // Count the marker motion up with one
    if(counter > (markerMotions.size() - 1))
        counter = 0;
    else
        counter++;

    // Update the state
    getRobWorkStudio()->setState(state);


	if (_framegrabber != NULL) {
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
        Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

        //cv::imwrite("/home/student/Desktop/test.png", imflip);

        Vision vision;
        LineFinding line_finder(imflip);
        auto markers = line_finder.get_marker_points(&imflip);
        for(uint8_t i = 0; i < markers.size(); i++)
        {
            cv::circle(imflip, markers[i],  5, Scalar( (i * 1000) % 256,i * 60,255 - i * 60),  CV_FILLED);
        }

        //Mat trackedImg = vision.trackPicture(imflip);
        Mat trackedImg = imflip;
        //Mat trackedImg = trackPicture(imflip);

		// Show in QLabel
        QImage img(trackedImg.data, trackedImg.cols, trackedImg.rows, trackedImg.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}
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

Q_EXPORT_PLUGIN(SamplePlugin);
