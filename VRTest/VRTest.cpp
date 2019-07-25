#include "windows.h"
#include "osgStaticLibs.h"
#include <osgViewer/ViewerEventHandlers>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Image>
#include <osg/io_utils>
#include <sstream>
#include "OpenVRDevice.h"

class CameraCallback : public osg::Camera::DrawCallback
{
public:
	CameraCallback(osg::Texture2D *lt, OpenVRDevice *device) : m_device(device), leftTexture(lt) {}
	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		int cID = renderInfo.getContextID();
		auto textureObject = leftTexture->getTextureObject(cID);
		m_device->m_leftTextureID = (unsigned int)textureObject->id();
		m_device->submitFrame();
	}
private:
	osg::ref_ptr<OpenVRDevice> m_device;
	osg::ref_ptr<osg::Texture2D> leftTexture;
};

class CameraMovement : public osg::NodeCallback
{
public:
	CameraMovement(OpenVRDevice *device) : m_device(device)
	{
		eye = osg::Vec3(1.0, 0.0, 0.0);
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		m_device->updateHMDMatrixPose();
		theta += 1.2;
		float a = theta / 180 * 3.1415926;
		eye = osg::Vec3(cos(a), sin(a), 0.0) * 10.0;
		node->asCamera()->setViewMatrixAsLookAt(eye, osg::Vec3(0.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));
		osg::Vec3 t, c, u;
		node->asCamera()->getViewMatrixAsLookAt(t, c, u);
		std::stringstream ss;
		ss << t << std::endl;
		OutputDebugStringA(ss.str().c_str());
	}
private:
	osg::ref_ptr<OpenVRDevice> m_device;
	osg::Vec3 eye;
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

//class OpenVRUpdateSlaveCallback : public osg::View::Slave::UpdateSlaveCallback
//{
//public:
//	enum CameraType
//	{
//		LEFT_CAMERA,
//		RIGHT_CAMERA
//	};
//
//	OpenVRUpdateSlaveCallback(CameraType cameraType, OpenVRDevice *device, OpenVRSwapCallback* swapCallback) :
//		m_cameraType(cameraType),
//		m_swapCallback(swapCallback) {}
//
//	virtual void updateSlave(osg::View& view, osg::View::Slave& slave)
//	{
//		if (m_cameraType == LEFT_CAMERA)
//		{
//			m_device->updateHMDMatrixPose();
//		}
//
//		/*osg::Vec3 position = m_device->getHMDPosition();
//		osg::Quat orientation = m_device->getHMDOrientation();
//
//		osg::Matrix viewOffset = (m_cameraType == LEFT_CAMERA) ? m_device->getViewMatrixLeft() : m_device->getViewMatrixRight();
//
//		viewOffset.preMultRotate(orientation);
//		viewOffset.setTrans(viewOffset.getTrans() + position);
//
//		slave._viewOffset = viewOffset;
//		slave.updateSlaveImplementation(view);*/
//	}
//	CameraType m_cameraType;
//	osg::ref_ptr<OpenVRDevice> m_device;
//	osg::ref_ptr<OpenVRSwapCallback> m_swapCallback;
//};

int main()
{
	OpenVRDevice* openvrDevice = new OpenVRDevice;
	//Æô¶¯OpenVR
	if (!openvrDevice->openVRInit())
	{
		openvrDevice->openVRShutdown();
	}

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile("cow.osgt");
	osg::ref_ptr<osg::MatrixTransform> cowPosition = new osg::MatrixTransform;
	cowPosition->addChild(cow);
	osg::Matrix cowPos;
	cowPos.setTrans(osg::Vec3(10.0, 0.0, 0.0));
	cowPosition->setMatrix(cowPos);

	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::ref_ptr<osg::Camera> leftCamera = new osg::Camera;
	osg::ref_ptr<osg::Camera> rightCamera = new osg::Camera;

	osg::ref_ptr<osg::Image> image = osgDB::readImageFile("reflect.rgb");

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	texture->setTextureSize(1024, 1024);
	texture->setInternalFormat(GL_RGBA);
	texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
	texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	//texture->setImage(image);

	rightCamera->setViewport(0, 0, 1024, 1024);
	rightCamera->setProjectionMatrixAsPerspective(30.0, 1.6667, 0.1, 50.0);
	rightCamera->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
	rightCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	rightCamera->setRenderOrder(osg::Camera::PRE_RENDER);
	rightCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	rightCamera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	rightCamera->setUpdateCallback(new CameraMovement(openvrDevice));
	rightCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	rightCamera->attach(osg::Camera::COLOR_BUFFER0, texture.get(), 0, 0, false, 4, 4);
	rightCamera->setFinalDrawCallback(new CameraCallback(texture, openvrDevice));
	//leftCamera->setViewMatrixAsLookAt(osg::Vec3(5.0f, 0.0f, 0.0f), osg::Vec3(6.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> leftPlane = osg::createTexturedQuadGeometry(osg::Vec3(), osg::Vec3(5.0, 0, 0), osg::Vec3(0, 0, 5));
	osg::ref_ptr<osg::StateSet> ss = new osg::StateSet();
	ss->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);
	leftPlane->setStateSet(ss.get());
	//leftCamera->attach(osg::Camera::COLOR_BUFFER, leftTexture);
	
	
	root->addChild(leftPlane);
	root->addChild(cowPosition);
	root->addChild(rightCamera);
	rightCamera->addChild(cow);

	viewer.setUpViewInWindow(10, 10, 1024, 768);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	
	viewer.setSceneData(root);
	//viewer.addSlave(rightCamera.get());
	//viewer.addSlave(leftCamera);
	//viewer.addSlave(rightCamera);

	//viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(0.5f, -1.0f, 0.5f), osg::Vec3(0.5f, 1.0f, 0.5f), osg::Vec3(0.0f, 0.0f, 1.0f));
	osg::ref_ptr<osg::GraphicsContext> gc = viewer.getCamera()->getGraphicsContext();
	osg::ref_ptr<OpenVRSwapCallback> swapCallback = new OpenVRSwapCallback(openvrDevice);
	gc->setSwapCallback(swapCallback);

	
	return viewer.run();
	while (!viewer.done())
	{
		viewer.frame();
	}
	return 0;
}