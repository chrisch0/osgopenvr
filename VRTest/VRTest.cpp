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
#define PI 3.1415926
enum Eye
{
	Left = 0,
	Right
};

class CameraCallback : public osg::Camera::DrawCallback
{
public:
	CameraCallback(Eye eye, osg::Texture2D *texture, OpenVRDevice *device) : m_eye(eye), m_device(device), m_texture(texture) {}
	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		int cID = renderInfo.getContextID();
		auto textureObject = m_texture->getTextureObject(cID);
		if (m_eye == Left)
		{
			m_device->m_leftTextureID = (unsigned int)textureObject->id();
			m_device->submitLeftEyeFrame();
		}
		if (m_eye == Right)
		{
			m_device->m_rightTextureID = (unsigned int)textureObject->id();
			m_device->submitRightEyeFrame();
		}
		
	}
private:
	Eye m_eye;
	osg::ref_ptr<OpenVRDevice> m_device;
	osg::ref_ptr<osg::Texture2D> m_texture;
};

class OpenVRUpdateCallback : public osg::NodeCallback
{
public:
	OpenVRUpdateCallback(Eye e, OpenVRDevice *device, std::shared_ptr<osg::Vec3> refpos, float *h_angle) : m_device(device), m_eye(e), m_refPos(refpos)
	{
		eye = osg::Vec3(1.0, 0.0, 0.0);
		m_head = h_angle;
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (m_eye == Left)
		{
			m_device->updateHMDMatrixPose();
		}
		osg::Matrix ref;
		ref.makeRotate(*m_head * 0.017453, osg::Vec3(0.0, 0.0, 1.0));
		ref.setTrans(*m_refPos);
		if (m_eye == Left)
		{
			osg::Matrix m = osg::Matrix::inverse(ref) * m_device->getLeftEyeViewMatrix();
			node->asCamera()->setViewMatrix(osg::Matrix::inverse(ref) * m_device->getLeftEyeViewMatrix());
			node->asCamera()->setProjectionMatrix(m_device->getProjectionMatrixLeft());
		}
		if (m_eye == Right)
		{
			node->asCamera()->setViewMatrix(osg::Matrix::inverse(ref) * m_device->getRightEyeViewMatrix());
			node->asCamera()->setProjectionMatrix(m_device->getProjectionMatrixRight());
		}
		m_device->setControllerMatrixTransform(m_eye, node->asCamera()->getViewMatrix());
		traverse(node, nv);
	}
private:
	Eye m_eye;
	osg::ref_ptr<OpenVRDevice> m_device;
	osg::Vec3 eye;
	std::shared_ptr<osg::Vec3> m_refPos;
	float *m_head;
	float theta = 0.0;
};

class OpenVRSwapCallback : public osg::GraphicsContext::SwapCallback
{
public:
	OpenVRSwapCallback(OpenVRDevice* device) : m_device(device), m_frameIndex(0) {}
	void swapBuffersImplementation(osg::GraphicsContext* gc)
	{
		gc->swapBuffersImplementation();
	}
	int frameIndex() const { return m_frameIndex; }
private:
	osg::observer_ptr<OpenVRDevice> m_device;
	int m_frameIndex;
};

osg::Camera* createRTTCamera(Eye eye, OpenVRDevice *device, osg::ref_ptr<osg::Texture2D> texture, std::shared_ptr<osg::Vec3> refPos, float *head)
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;

	camera->setViewport(0, 0, device->getTextureWidth(), device->getTextureHeight());
	if (eye == Left)
		camera->setProjectionMatrix(device->getProjectionMatrixLeft());
	if (eye == Right)
		camera->setProjectionMatrix(device->getProjectionMatrixRight());
	camera->setClearColor(osg::Vec4(0.2f, 0.0f, 0.8f, 0.0f));
	camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camera->setRenderOrder(osg::Camera::PRE_RENDER, eye);
	camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	camera->setUpdateCallback(new OpenVRUpdateCallback(eye, device, refPos, head));
	//camera->setUpdateCallback(new CameraMovement(device));
	camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	camera->attach(osg::Camera::COLOR_BUFFER0, texture, 0, 0, false, 4, 4);
	camera->setFinalDrawCallback(new CameraCallback(eye, texture, device));
	return camera.release();
}

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
	osg::ref_ptr<osg::Camera> leftCamera = createRTTCamera(Left, openvrDevice, leftTexture, camPos, &head);
	osg::ref_ptr<osg::Camera> rightCamera = createRTTCamera(Right, openvrDevice, rightTexture, camPos, &head);


	osg::ref_ptr<osg::Geometry> leftPlane = osg::createTexturedQuadGeometry(osg::Vec3(0.0, 0.0, 0.0), osg::Vec3(0.5, 0, 0), osg::Vec3(0, 1.0, 0));
	osg::ref_ptr<osg::StateSet> lss = new osg::StateSet;
	lss->setTextureAttributeAndModes(0, leftTexture, osg::StateAttribute::ON);
	leftPlane->setStateSet(lss);
	
	osg::ref_ptr<osg::Geometry> rightPlane = osg::createTexturedQuadGeometry(osg::Vec3(0.5, 0.0, 0.0), osg::Vec3(0.5, 0, 0), osg::Vec3(0, 1.0, 0));
	osg::ref_ptr<osg::StateSet> rss = new osg::StateSet;
	rss->setTextureAttributeAndModes(0, rightTexture, osg::StateAttribute::ON);
	rightPlane->setStateSet(rss);
	
	scene->addChild(cowMatrixTransform);
	scene->addChild(openvrDevice->getControllerNode(0));
	scene->addChild(openvrDevice->getControllerNode(1));
	leftCamera->addChild(scene);
	rightCamera->addChild(scene);
	
	root->addChild(leftPlane);
	root->addChild(rightPlane);
	root->addChild(leftCamera);
	root->addChild(rightCamera);

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