#include "windows.h"
#include "osgStaticLibs.h"
#include <osgViewer/ViewerEventHandlers>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Image>

//class CameraCallback : osg::Camera::DrawCallback
//{
//	virtual void operator () (osg::RenderInfo& renderInfo) 
//	{
//		int cID = renderInfo.getContextID();
//		auto textureObject = leftTexture->getTextureObject(cID);
//		textureObject->id()
//	}
//	osg::ref_ptr<osg::Texture2D> leftTexture = new osg::Texture2D;
//};

int main()
{
	//Æô¶¯OpenVR

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
	rightCamera->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
	rightCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//rightCamera->setViewMatrixAsLookAt(osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(6.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
	rightCamera->setRenderOrder(osg::Camera::PRE_RENDER);
	rightCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	rightCamera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);

	osg::ref_ptr<osg::Image> ri = new osg::Image;
	ri->allocateImage(1024, 1024, 1, GL_RGBA, GL_FLOAT);
	rightCamera->attach(osg::Camera::COLOR_BUFFER, texture.get());

	//leftCamera->setViewMatrixAsLookAt(osg::Vec3(5.0f, 0.0f, 0.0f), osg::Vec3(6.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> leftPlane = osg::createTexturedQuadGeometry(osg::Vec3(), osg::Vec3(1.0, 0, 0), osg::Vec3(0, 0, 1));
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
	//viewer.addSlave(leftCamera);
	//viewer.addSlave(rightCamera);

	//viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(0.5f, -1.0f, 0.5f), osg::Vec3(0.5f, 1.0f, 0.5f), osg::Vec3(0.0f, 0.0f, 1.0f));
	return viewer.run();
	while (!viewer.done())
	{
		viewer.frame();
	}
	return 0;
}