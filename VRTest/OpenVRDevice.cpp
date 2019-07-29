#include "OpenVRDevice.h"
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <sstream>


#ifndef MAX_UNICODE_PATH
#define MAX_UNICODE_PATH 32767
#endif

#ifndef MAX_UNICODE_PATH_IN_UTF8
#define MAX_UNICODE_PATH_IN_UTF8 (MAX_UNICODE_PATH * 4)
#endif

#ifndef PI
#define PI 3.1415926
#endif

const std::string OpenVRDevice::m_version = "0.0.3";

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

bool GetDigitalActionRisingEdge(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr)
{
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
	if (pDevicePath)
	{
		*pDevicePath = vr::k_ulInvalidInputValueHandle;
		if (actionData.bActive)
		{
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				*pDevicePath = originInfo.devicePath;
			}
		}
	}
	return actionData.bActive && actionData.bChanged && actionData.bState;
}

bool GetDigitalActionState(vr::VRActionHandle_t action, vr::VRInputValueHandle_t *pDevicePath = nullptr)
{
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(action, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
	if (pDevicePath)
	{
		*pDevicePath = vr::k_ulInvalidInputValueHandle;
		if (actionData.bActive)
		{
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(actionData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				*pDevicePath = originInfo.devicePath;
			}
		}
	}
	return actionData.bActive && actionData.bState;
}

std::string getExecutablePath()
{
	wchar_t *pwchPath = new wchar_t[MAX_UNICODE_PATH];
	char *pchPath = new char[MAX_UNICODE_PATH_IN_UTF8];
	::GetModuleFileNameW(NULL, pwchPath, MAX_UNICODE_PATH);
	WideCharToMultiByte(CP_UTF8, 0, pwchPath, -1, pchPath, MAX_UNICODE_PATH_IN_UTF8, NULL, NULL);
	delete[] pwchPath;

	std::string sPath = pchPath;
	delete[] pchPath;
	return sPath;
}

std::string getAbsolutePath(const std::string & relativePath)
{
	std::string executablePath = getExecutablePath();
	std::string::size_type n = executablePath.find_last_of("\\");
	if (n == std::string::npos)
		return executablePath + relativePath;
	else
		return std::string(executablePath.begin(), executablePath.begin() + n) + relativePath;
}

std::string OpenVRDevice::getTrackedDeviceInfo(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
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

OpenVRDevice::OpenVRDevice(float nearClip, float farClip, const float worldUnitsPerMetre /* = 1.0f */, const int sample /* = 0 */) :
	m_vrSystem(nullptr),
	m_vrRenderModels(nullptr),
	m_worldUnitsPerMetre(worldUnitsPerMetre),
	m_hmdPosition(osg::Vec3(0.f, 0.f, 0.f)),
	m_hmdOrientation(osg::Quat(0.f, 0.f, 0.f, 1.f)),
	m_nearClip(nearClip),
	m_farClip(farClip),
	m_samples(sample)
{
	trySetProcessAsHighPriority();
	m_openVRMatrixtoOsgMatrix.makeRotate(-0.5*PI, osg::Vec3(1.0, 0.0, 0.0));
}

OpenVRDevice::~OpenVRDevice()
{

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

	std::string driverName = getTrackedDeviceInfo(vr::Prop_TrackingSystemName_String);
	std::string deviceSerialNumber = getTrackedDeviceInfo(vr::Prop_SerialNumber_String);
	osg::notify(osg::NOTICE) << "HMD driver name: " << driverName << std::endl;
	osg::notify(osg::NOTICE) << "HMD device serial number: " << deviceSerialNumber << std::endl;

	std::string jsonPath = getAbsolutePath("./vrcontroller.json");
	vr::VRInput()->SetActionManifestPath(jsonPath.c_str());
	vr::VRInput()->GetActionHandle("/actions/controller/in/Trigger", &m_actionTrigger);
	vr::VRInput()->GetActionHandle("/actions/controller/in/Menu", &m_actionMenu);
	vr::VRInput()->GetActionHandle("/actions/controller/in/TriggerHaptic", &m_actionTriggerHaptic);
	vr::VRInput()->GetActionHandle("/actions/controller/in/AnalogInput", &m_actionAnalongInput);

	vr::VRInput()->GetActionSetHandle("/actions/controller", &m_actionsetController);

	vr::VRInput()->GetActionHandle("/actions/controller/out/Haptic_Left", &m_rHand[Left].m_actionHaptic);
	vr::VRInput()->GetInputSourceHandle("/user/hand/left", &m_rHand[Left].m_source);
	vr::VRInput()->GetActionHandle("/actions/controller/in/Hand_Left", &m_rHand[Left].m_actionPose);

	vr::VRInput()->GetActionHandle("/actions/controller/out/Haptic_Right", &m_rHand[Right].m_actionHaptic);
	vr::VRInput()->GetInputSourceHandle("/user/hand/right", &m_rHand[Right].m_source);
	vr::VRInput()->GetActionHandle("/actions/controller/in/Hand_Right", &m_rHand[Right].m_actionPose);
	
	prepareControllerModel();

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
	vr::VREvent_t event;
	while (m_vrSystem->PollNextEvent(&event, sizeof(event)))
	{
		//ProcessVREvent(event);
	}

	vr::VRActiveActionSet_t actionSet = { 0 };
	actionSet.ulActionSet = m_actionsetController;
	vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);

	m_rHand[Left].m_isTriggered = false;
	m_rHand[Right].m_isTriggered = false;

	vr::VRInputValueHandle_t ulTriggeredDevice;
	if (GetDigitalActionState(m_actionTrigger, &ulTriggeredDevice))
	{
		if (ulTriggeredDevice == m_rHand[Left].m_source)
		{
			m_rHand[Left].m_isTriggered = true;
		}
		if (ulTriggeredDevice == m_rHand[Right].m_source)
		{
			m_rHand[Right].m_isTriggered = true;
		}
	}


	vr::VRInputValueHandle_t ulHapticDevice;
	if (GetDigitalActionRisingEdge(m_actionTriggerHaptic, &ulHapticDevice))
	{
		if (ulHapticDevice == m_rHand[Left].m_source)
		{
			vr::VRInput()->TriggerHapticVibrationAction(m_rHand[Left].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle);
		}
		if (ulHapticDevice == m_rHand[Right].m_source)
		{
			vr::VRInput()->TriggerHapticVibrationAction(m_rHand[Right].m_actionHaptic, 0, 1, 4.f, 1.0f, vr::k_ulInvalidInputValueHandle);
		}
	}

	vr::InputAnalogActionData_t analogData;
	if (vr::VRInput()->GetAnalogActionData(m_actionAnalongInput, &analogData, sizeof(analogData), vr::k_ulInvalidInputValueHandle) == vr::VRInputError_None && analogData.bActive)
	{
		m_vAnalogValue[0] = analogData.x;
		m_vAnalogValue[1] = analogData.y;
	}

	m_rHand[Left].m_menu = true;
	m_rHand[Right].m_menu = true;

	vr::VRInputValueHandle_t ulHideDevice;
	if (GetDigitalActionState(m_actionMenu, &ulHideDevice))
	{
		if (ulHideDevice == m_rHand[Left].m_source)
		{
			m_rHand[Left].m_menu = false;
		}
		if (ulHideDevice == m_rHand[Right].m_source)
		{
			m_rHand[Right].m_menu = false;
		}
	}

	for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
	{
		vr::InputPoseActionData_t poseData;
		if (vr::VRInput()->GetPoseActionData(m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, 0, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle) != vr::VRInputError_None
			|| !poseData.bActive || !poseData.pose.bPoseIsValid)
		{
			m_rHand[eHand].m_menu = false;
		}
		else
		{
			osg::Matrix matrix = convertMatrix34(poseData.pose.mDeviceToAbsoluteTracking);
			m_rHand[eHand].m_matrix = matrix;
			m_rHand[eHand].m_position = matrix.getTrans();
			m_rHand[eHand].m_orientation = matrix.getRotate();
		}
	}
}

void OpenVRDevice::updateHMDMatrixPose()
{
	vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);

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
		m_hmdPoseMatrix = matrix;
		osg::Matrix poseTransform = osg::Matrix::inverse(matrix);
		m_hmdPosition = poseTransform.getTrans() * m_worldUnitsPerMetre;
		m_hmdOrientation = poseTransform.getRotate();
		m_hmdPoseInverseMatrix = poseTransform;
	}

	handleInput();
}

void OpenVRDevice::configureCamera()
{
	calculateEyeAdjustment();
	calculateProjectionMatrices();
	m_vrSystem->GetRecommendedRenderTargetSize(&m_textureWidth, &m_textureHeight);
}

bool OpenVRDevice::submitFrame()
{
	vr::Texture_t leftEyeTexture = { (void*)m_leftTextureID,  vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
	vr::Texture_t rightEyeTexture = { (void*)m_rightTextureID, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

	vr::EVRCompositorError lError = vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
	vr::EVRCompositorError rError = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

	return lError == vr::VRCompositorError_None && rError == vr::VRCompositorError_None;
}

bool OpenVRDevice::submitLeftEyeFrame()
{
	vr::Texture_t leftEyeTexture = { (void*)m_leftTextureID,  vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

	vr::EVRCompositorError lError = vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);

	return lError == vr::VRCompositorError_None;
}

bool OpenVRDevice::submitRightEyeFrame()
{
	vr::Texture_t rightEyeTexture = { (void*)m_rightTextureID, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

	vr::EVRCompositorError rError = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

	return rError == vr::VRCompositorError_None;
}

void OpenVRDevice::resetSensorOrientation() const
{
	m_vrSystem->ResetSeatedZeroPose();
}

void OpenVRDevice::setControllerMatrixTransform(int eye, osg::Matrix viewMatrix)
{
	for (int i = 0; i < 2; ++i)
	{
		osg::Matrix matController = getControllerMatrix(i);
	
		osg::Matrix inversedViewMatrix = osg::Matrix::inverse(viewMatrix);
		osg::Matrix eyeMatrix = (eye == 0) ? m_leftEyePosInverseMatrix : m_rightEyePosInverseMatrix;
		osg::Matrix viewTrans = m_hmdPoseInverseMatrix * eyeMatrix * inversedViewMatrix;
		osg::Matrix controller = m_openVRMatrixtoOsgMatrix * matController * viewTrans;
		osg::Matrix worldMatrix = m_openVRMatrixtoOsgMatrix * matController * m_hmdPoseInverseMatrix * inversedViewMatrix;
		m_rHand[i].m_worldPosition = osg::Vec3(0.0, 0.0, 0.0) * worldMatrix;
		m_rHand[i].m_worldTowards = osg::Vec3(0.0, m_lineLength, 0.0) * worldMatrix;
		getControllerNode(i)->setMatrix(controller);
		osg::Matrix m = eyeMatrix * osg::Matrix::inverse(viewMatrix);
		getHMDLookAtPoint()->setMatrix(m);
	}
}

bool OpenVRDevice::hmdPresent()
{
	return vr::VR_IsHmdPresent();
}

bool OpenVRDevice::hmdInitialized() const
{
	return m_vrSystem != nullptr && m_vrRenderModels != nullptr;
}

osg::Matrix OpenVRDevice::getProjectionOffsetMatrixLeft() const 
{
	osg::Matrix projectionOffsetMatrix;
	float offset = m_leftEyeProjectionMatrix(2, 0);
	projectionOffsetMatrix.makeTranslate(osg::Vec3(-offset, 0.0, 0.0));
	return projectionOffsetMatrix;
}

osg::Matrix OpenVRDevice::getProjectionOffsetMatrixRight() const
{
	osg::Matrix projectionOffsetMatrix;
	float offset = m_rightEyeProjectionMatrix(2, 0);
	projectionOffsetMatrix.makeTranslate(osg::Vec3(-offset, 0.0, 0.0));
	return projectionOffsetMatrix;
}

osg::Matrix OpenVRDevice::getViewMatrixLeft() const 
{
	osg::Matrix viewMatrix;
	viewMatrix.makeTranslate(-m_leftEyeAdjust);
	return viewMatrix;
}

osg::Matrix OpenVRDevice::getViewMatrixRight() const
{
	osg::Matrix viewMatrix;
	viewMatrix.makeTranslate(-m_rightEyeAdjust);
	return viewMatrix;
}

void OpenVRDevice::prepareControllerModel()
{
	// Load controller model
	osg::ref_ptr<osg::Node> controller = osgDB::readNodeFile("data/vivecontroller.ive");
	if (controller == nullptr)
	{
		osg::notify(osg::WARN) << "Cannot find VR controller model" << std::endl;
	}

	m_controllers[0] = new osg::MatrixTransform;
	m_controllers[1] = new osg::MatrixTransform;

	m_controllers[0]->addChild(controller);
	m_controllers[1]->addChild(controller);

	// Add controller towards line
	osg::ref_ptr<osg::Geometry> towards = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vlist = new osg::Vec3Array;
	vlist->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vlist->push_back(osg::Vec3(0.0, m_lineLength, 0.0));
	towards->setVertexArray(vlist);

	osg::ref_ptr<osg::Vec4Array> clist = new osg::Vec4Array;
	clist->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
	clist->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
	towards->setColorArray(clist, osg::Array::BIND_PER_VERTEX);

	towards->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vlist->size()));
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(towards);

	m_controllers[0]->addChild(geode);
	m_controllers[1]->addChild(geode);

	// Add look at line
	osg::ref_ptr<osg::Geometry> line = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> linelist = new osg::Vec3Array;
	linelist->push_back(osg::Vec3(0.0, 0.0, -0.3));
	linelist->push_back(osg::Vec3(0.0, 0.0, -m_lineLength));
	line->setVertexArray(linelist);
	line->setColorArray(clist, osg::Array::BIND_PER_VERTEX);
	line->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, linelist->size()));

	// Add look at point
	m_lookAtPoint = new osg::MatrixTransform;
	osg::ref_ptr<osg::Geode> lookat = new osg::Geode;
	osg::TessellationHints *hints = new osg::TessellationHints();
	hints->setDetailRatio(0.5);
	lookat->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0, 0.0, -1.0), 0.01f), hints));
	lookat->addChild(line);

	m_lookAtPoint->addChild(lookat);

	if (!m_isShowLines)
	{
		//geode->asGroup()->getChild(0)->setNodeMask(0);
		for (int i = 0; i < lookat->asGroup()->getNumChildren(); ++i)
		{
			lookat->asGroup()->getChild(i)->setNodeMask(0);
		}
	}
}

void OpenVRDevice::calculateEyeAdjustment()
{
	osg::Matrix mat;
	mat = convertMatrix34(m_vrSystem->GetEyeToHeadTransform(vr::Eye_Left));
	m_leftEyePosMatrix = mat;
	m_leftEyeAdjust = mat.getTrans();
	m_leftEyePosInverseMatrix = osg::Matrix::inverse(mat);
	mat = convertMatrix34(m_vrSystem->GetEyeToHeadTransform(vr::Eye_Right));
	m_rightEyePosMatrix = mat;
	m_rightEyeAdjust = mat.getTrans();
	m_rightEyePosInverseMatrix = osg::Matrix::inverse(mat);

	// Display IPD
	float ipd = (m_leftEyeAdjust - m_rightEyeAdjust).length();
	osg::notify(osg::ALWAYS) << "Interpupillary Distance (IPD): " << ipd * 1000.0f << " mm" << std::endl;

	// Scale to world units
	m_leftEyeAdjust *= m_worldUnitsPerMetre;
	m_rightEyeAdjust *= m_worldUnitsPerMetre;
}

void OpenVRDevice::calculateProjectionMatrices()
{
	vr::HmdMatrix44_t mat;

	mat = m_vrSystem->GetProjectionMatrix(vr::Eye_Left, m_nearClip, m_farClip);
	m_leftEyeProjectionMatrix = convertMatrix44(mat);

	mat = m_vrSystem->GetProjectionMatrix(vr::Eye_Right, m_nearClip, m_farClip);
	m_rightEyeProjectionMatrix = convertMatrix44(mat);
}

void OpenVRDevice::trySetProcessAsHighPriority() const
{
	// Require at least 4 processors, otherwise the process could occupy the machine.
	if (OpenThreads::GetNumberOfProcessors() >= 4)
	{
#ifdef _WIN32
		SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
	}
}