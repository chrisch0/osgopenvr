#pragma once
#include <windows.h>
#include <osg/Vec3>
#include <osg/Quat>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <array>
#include <openvr.h>

class OpenVRDevice : public osg::Referenced
{
public:
	std::string version() { return m_version; }

	OpenVRDevice(float nearClip, float farClip, const float worldUnitsPerMetre = 1.0f, const int sample = 0);
	OpenVRDevice(const OpenVRDevice&) = delete;
	OpenVRDevice& operator=(const OpenVRDevice&) = delete;

	typedef enum Eye_
	{
		LEFT = 0,
		RIGHT = 1,
		COUNT = 2
	} Eye;

	bool openVRInit();
	void openVRShutdown();
	void handleInput();
	void updateHMDMatrixPose();
	void configureCamera();
	bool submitFrame();
	bool submitLeftEyeFrame();
	bool submitRightEyeFrame();
	void resetSensorOrientation() const;

	static bool hmdPresent();
	bool hmdInitialized() const;

	//void creatRenderBuffers(osg::ref_ptr<osg::State> state);
	//osg::Camera* createRTTCamera(OpenVRDevice::Eye)

	void setControllerMatrixTransform(osg::Matrix viewMatrix, int eye);

	osg::Vec3 getHMDPosition() const { return m_hmdPosition; }
	osg::Quat getHMDOrientation() const { return m_hmdOrientation; }

	osg::Matrix getProjectionMatrixCenter() const { return (m_leftEyeProjectionMatrix + m_rightEyeProjectionMatrix) * 0.5; }
	osg::Matrix getProjectionMatrixLeft() const { return m_leftEyeProjectionMatrix; }
	osg::Matrix getProjectionMatrixRight() const { return m_rightEyeProjectionMatrix; }
	osg::Matrix getProjectionOffsetMatrixLeft() const;
	osg::Matrix getProjectionOffsetMatrixRight() const;

	osg::Matrix getViewMatrixLeft() const;
	osg::Matrix getViewMatrixRight() const;

	float getNearClip() const { return m_nearClip; }
	float getFarClip() const { return m_farClip; }

	uint32_t getTextureWidth() const { return m_textureWidth; }
	uint32_t getTextureHeight() const { return m_textureHeight; }

	osg::Vec3 getControllerPosition(int index) const
	{
		if (index >= 0 && index <= 1) return m_rHand[index].m_worldPosition;
		else return m_rHand[0].m_worldPosition;
	}
	//index = 0, left; index = 1, right
	//Get the controller towards
	osg::Vec3 getControllerTowards(int index) const
	{
		if (index >= 0 && index <= 1) return m_rHand[index].m_worldTowards - getControllerPosition(index);
		else return m_rHand[0].m_worldTowards - getControllerPosition(0);
	}
	osg::Matrix getControllerMatrix(int index) const
	{
		if (index >= 0 && index <= 1) return m_rHand[index].m_matrix;
		else return m_rHand[0].m_matrix;
	}
	//index = 0, left; index = 1, right
	//Get the controller node
	osg::ref_ptr<osg::MatrixTransform> getControllerNode(int index) const
	{
		if (index >= 0 && index <= 1) return m_controllers[index];
		else return m_controllers[0];
	}
	osg::ref_ptr<osg::MatrixTransform> getHMDLookAtPoint() const
	{
		return m_lookAtPoint;
	}
	//index = 0, left; index = 1, right
	//Return true while the controller trigger is pulled
	bool isControllerTriggered(int index) const
	{
		if (index == 0) return m_rHand[Left].m_isTriggered;
		if (index == 1) return m_rHand[Right].m_isTriggered;
		return false;
	}

	unsigned int m_leftTextureID = 0;
	unsigned int m_rightTextureID = 0;
	bool m_isShowLines;
protected:
	~OpenVRDevice();

private:
	std::string getTrackedDeviceInfo(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL);
	void prepareControllerModel();

	void calculateEyeAdjustment();
	void calculateProjectionMatrices();

	void trySetProcessAsHighPriority() const;

	static const std::string m_version;

	vr::IVRSystem *m_vrSystem;
	vr::IVRRenderModels *m_vrRenderModels;

	osg::Matrixf m_leftEyeProjectionMatrix;
	osg::Matrixf m_rightEyeProjectionMatrix;
	osg::Vec3f m_leftEyeAdjust;
	osg::Vec3f m_rightEyeAdjust;

	float m_nearClip;
	float m_farClip;
	int m_samples;

	uint32_t m_textureWidth;
	uint32_t m_textureHeight;

	osg::Vec3 m_hmdPosition;
	osg::Quat m_hmdOrientation;

	float m_worldUnitsPerMetre;

	std::array<osg::ref_ptr<osg::MatrixTransform>, 2> m_controllers;
	float m_lineLength = 1.0;

	osg::ref_ptr<osg::MatrixTransform> m_lookAtPoint;

	//inversed
	osg::Matrix m_hmdPoseMatrix;
	//inversed
	osg::Matrix m_leftEyePosMatrix;
	//inversed
	osg::Matrix m_rightEyePosMatrix;

	float m_vAnalogValue[2];
	struct ControllerInfo_t
	{
		vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
		vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
		vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
		osg::Matrix m_matrix;
		osg::Vec3 m_position;
		osg::Quat m_orientation;
		osg::Vec3 m_worldPosition;
		osg::Vec3 m_worldTowards;
		bool m_isTriggered;
		bool m_menu;
	};

	enum EHand
	{
		Left = 0,
		Right = 1,
	};
	ControllerInfo_t m_rHand[2];

	vr::VRActionHandle_t m_actionMenu = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionTrigger = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionTriggerHaptic = vr::k_ulInvalidActionHandle;
	vr::VRActionHandle_t m_actionAnalongInput = vr::k_ulInvalidActionHandle;

	vr::VRActionSetHandle_t m_actionsetController = vr::k_ulInvalidActionSetHandle;
};