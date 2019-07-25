#include <osg/io_utils>
#include <sstream>
#include "OpenVRDevice.h"

static osg::Matrix convertMatrix34(const vr::HmdMatrix34_t &mat34)
{
	osg::Matrix matrix(
		mat34.m[0][0], mat34.m[1][0], mat34.m[2][0], 0.0,
		mat34.m[0][1], mat34.m[1][1], mat34.m[2][1], 0.0,
		mat34.m[0][2], mat34.m[1][2], mat34.m[2][2], 0.0,
		mat34.m[0][3], mat34.m[1][3], mat34.m[2][3], 1.0f
	);
	return matrix;
}

static osg::Matrix convertMatrix44(const vr::HmdMatrix44_t &mat44)
{
	osg::Matrix matrix(
		mat44.m[0][0], mat44.m[1][0], mat44.m[2][0], mat44.m[3][0],
		mat44.m[0][1], mat44.m[1][1], mat44.m[2][1], mat44.m[3][1],
		mat44.m[0][2], mat44.m[1][2], mat44.m[2][2], mat44.m[3][2],
		mat44.m[0][3], mat44.m[1][3], mat44.m[2][3], mat44.m[3][3]
	);
	return matrix;
}

std::string OpenVRDevice::GetTrackedDeviceInfo(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
{
	uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";

	char *pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;
	return sResult;
}

bool OpenVRDevice::openVRInit()
{
	vr::EVRInitError eError = vr::VRInitError_None;
	m_vrSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);

	if (eError != vr::VRInitError_None)
	{
		m_vrSystem = nullptr;
		osg::notify(osg::WARN)
			<< "Error : VR_Init Failed, unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
		return false;
	}

	if (!vr::VRCompositor())
	{
		m_vrSystem = nullptr;
		vr::VR_Shutdown();
		osg::notify(osg::WARN)
			<< "Error: Compositor initialization failed" << std::endl;
		return false;
	}

	m_vrRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
	if (m_vrRenderModels == nullptr)
	{
		m_vrSystem = nullptr;
		vr::VR_Shutdown();
		osg::notify(osg::WARN)
			<< "Error: Unable to get render model interface: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
		return false;
	}

	std::string driverName = GetTrackedDeviceInfo(vr::Prop_TrackingSystemName_String);
	std::string deviceSerialNumber = GetTrackedDeviceInfo(vr::Prop_SerialNumber_String);
	osg::notify(osg::NOTICE) << "HMD driver name: " << driverName << std::endl;
	osg::notify(osg::NOTICE) << "HMD device serial number: " << deviceSerialNumber << std::endl;

	if (!m_vrSystem)
	{
		//controller
	}
	return true;
}

void OpenVRDevice::openVRShutdown()
{
	if (m_vrSystem)
	{
		vr::VR_Shutdown();
		m_vrSystem = nullptr;
	}
}

void OpenVRDevice::handleInput()
{

}

void OpenVRDevice::updateHMDMatrixPose()
{
	vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseSeated);

	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) poses[i].bPoseIsValid = false;
	vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, NULL, 0);

	// Not sure why, but the openvr hellovr_opengl example only seems interested in the
	// pose transform from the first pose tracking device in the array.
	// i.e. this seems to be the only one that is used to affect the view transform matrix.
	// So, here we do the same.
	const vr::TrackedDevicePose_t& pose = poses[vr::k_unTrackedDeviceIndex_Hmd];
	if (pose.bPoseIsValid)
	{
		osg::Matrix matrix = convertMatrix34(pose.mDeviceToAbsoluteTracking);
		osg::Matrix poseTransform = osg::Matrix::inverse(matrix);
		m_hmdPosition = poseTransform.getTrans();
		m_hmdOrientation = poseTransform.getRotate();
	}
}

bool OpenVRDevice::submitFrame()
{
	vr::Texture_t leftEyeTexture = { (void*)m_leftTextureID,  vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
	vr::Texture_t rightEyeTexture = { (void*)m_leftTextureID, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

	vr::EVRCompositorError lError = vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
	vr::EVRCompositorError rError = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

	return lError == vr::VRCompositorError_None && rError == vr::VRCompositorError_None;
}