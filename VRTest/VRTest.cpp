#include "windows.h"
#include "osgStaticLibs.h"
#include <osgViewer/ViewerEventHandlers>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Image>
#include <osg/io_utils>
#include <sstream>
#include <cstdlib>
#include "OpenVRDevice.h"
#include "OpenVRCallback.h"
#define PI 3.1415926
enum Eye
{
	Left = 0,
	Right
};

osg::Texture2D* creatRenderTexture(OpenVRDevice* device)
{
	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	texture->setTextureSize(device->getTextureWidth(), device->getTextureHeight());
	texture->setInternalFormat(GL_RGBA);
	texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
	texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);

	return texture.release();
}

int main()
{
	if (!OpenVRDevice::hmdPresent())
	{
		osg::notify(osg::FATAL) << "Error: No valid HMD present!" << std::endl;
		return 1;
	}

	float nearClip = 0.1f;
	float farClip = 500.0f;

	osg::ref_ptr<OpenVRDevice> openvrDevice = new OpenVRDevice(nearClip, farClip);
	
	if (!openvrDevice->openVRInit())
	{
		openvrDevice->openVRShutdown();
	}

	if (!openvrDevice->hmdInitialized())
	{
		return 1;
	}

	openvrDevice->configureCamera();

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile("cow.osgt");
	osg::ref_ptr<osg::MatrixTransform> cowMatrixTransform = new osg::MatrixTransform;
	cowMatrixTransform->addChild(cow);
	osg::Matrix cowPos;
	cowPos.setTrans(osg::Vec3(0.0, 10.0, 0.0));
	cowMatrixTransform->setMatrix(cowPos);

	osg::ref_ptr<osg::Group> root = new osg::Group;
	osg::ref_ptr<osg::Group> scene = new osg::Group;

	osg::ref_ptr<osg::Image> image = osgDB::readImageFile("reflect.rgb");

	osg::ref_ptr<osg::Texture2D> leftTexture = creatRenderTexture(openvrDevice);
	osg::ref_ptr<osg::Texture2D> rightTexture = creatRenderTexture(openvrDevice);

	std::shared_ptr<osg::Vec3> camPos = std::make_shared<osg::Vec3>(0.0, 0.0, 0.0);
	float head = 0.0;
	openvrDevice->createRTTCamera(OpenVRDevice::Eye::LEFT, leftTexture, camPos, &head);
	openvrDevice->createRTTCamera(OpenVRDevice::Eye::RIGHT, rightTexture, camPos, &head);


	osg::ref_ptr<osg::Geometry> leftPlane = osg::createTexturedQuadGeometry(osg::Vec3(0.0, 0.0, 0.0), osg::Vec3(0.5, 0, 0), osg::Vec3(0, 1.0, 0));
	osg::ref_ptr<osg::StateSet> lss = new osg::StateSet;
	lss->setTextureAttributeAndModes(0, leftTexture, osg::StateAttribute::ON);
	leftPlane->setStateSet(lss);
	
	osg::ref_ptr<osg::Geometry> rightPlane = osg::createTexturedQuadGeometry(osg::Vec3(0.5, 0.0, 0.0), osg::Vec3(0.5, 0, 0), osg::Vec3(0, 1.0, 0));
	osg::ref_ptr<osg::StateSet> rss = new osg::StateSet;
	rss->setTextureAttributeAndModes(0, rightTexture, osg::StateAttribute::ON);
	rightPlane->setStateSet(rss);
	
	scene->addChild(cowMatrixTransform);
	openvrDevice->addChild(cowMatrixTransform);
	openvrDevice->addChild(openvrDevice->getControllerNode(0));
	openvrDevice->addChild(openvrDevice->getControllerNode(1));
	
	root->addChild(leftPlane);
	root->addChild(rightPlane);
	root->addChild(openvrDevice->getLeftCamera());
	root->addChild(openvrDevice->getRightCamera());

	viewer.setUpViewInWindow(100, 100, 1024, 512);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	
	viewer.setSceneData(root);
	viewer.getCamera()->setProjectionMatrixAsOrtho2D(0.0, 1.0, 0.0, 1.0);

	osg::ref_ptr<osg::GraphicsContext> gc = viewer.getCamera()->getGraphicsContext();
	osg::ref_ptr<OpenVRSwapCallback> swapCallback = new OpenVRSwapCallback(openvrDevice);
	gc->setSwapCallback(swapCallback);

	viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	
	//return viewer.run();
	while (!viewer.done())
	{
		*camPos = osg::Vec3(0, 10, 5);
		head = -90.0;
		viewer.frame();
	}
	return 0;
}