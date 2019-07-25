#pragma once
#include <osg/Vec3>
#include <osg/Quat>
#include <osg/Matrix>
#include <openvr.h>

class OpenVRDevice : public osg::Referenced
{
public:
	bool openVRInit();
	void openVRShutdown();
	void handleInput();
	void updateHMDMatrixPose();

	osg::Vec3 getHMDPosition() const { return m_hmdPosition; }
	osg::Quat getHMDOrientation() const { return m_hmdOrientation; }
	//osg::Matrix getViewMatrixLeft();
	//osg::Matrix getViewMatrixRight();

	bool submitFrame();

	unsigned int m_leftTextureID = 0;
private:
	std::string GetTrackedDeviceInfo(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL);
	vr::IVRSystem *m_vrSystem;
	vr::IVRRenderModels *m_vrRenderModels;
	osg::Vec3 m_hmdPosition;
	osg::Quat m_hmdOrientation;
	
	
};