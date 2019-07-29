#pragma once
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/Node>
#include <osg/RenderInfo>
#include "OpenVRDevice.h"

class OpenVRSubmitFrameCallback : public osg::Camera::DrawCallback
{
public:
	OpenVRSubmitFrameCallback(int eye, osg::Texture2D *texture, OpenVRDevice *device) : m_eye(eye), m_device(device), m_texture(texture) {}
	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		int cID = renderInfo.getContextID();
		auto textureObject = m_texture->getTextureObject(cID);
		if (m_eye == 0)
		{
			m_device->m_leftTextureID = (unsigned int)textureObject->id();
			m_device->submitLeftEyeFrame();
		}
		if (m_eye == 1)
		{
			m_device->m_rightTextureID = (unsigned int)textureObject->id();
			m_device->submitRightEyeFrame();
		}

	}
private:
	int m_eye;
	osg::ref_ptr<OpenVRDevice> m_device;
	osg::ref_ptr<osg::Texture2D> m_texture;
};

class OpenVRUpdateCallback : public osg::NodeCallback
{
public:
	OpenVRUpdateCallback(int e, OpenVRDevice *device, std::shared_ptr<osg::Vec3> refpos, float *h_angle) : m_device(device), m_eye(e), m_refPos(refpos)
	{
		m_head = h_angle;
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (m_eye == 0)
		{
			m_device->updateHMDMatrixPose();
		}
		osg::Matrix ref;
		ref.makeRotate(*m_head * 0.017453, osg::Vec3(0.0, 0.0, 1.0));
		ref.setTrans(*m_refPos);
		if (m_eye == 0)
		{
			osg::Matrix m = osg::Matrix::inverse(ref) * m_device->getLeftEyeViewMatrix();
			node->asCamera()->setViewMatrix(osg::Matrix::inverse(ref) * m_device->getLeftEyeViewMatrix());
			node->asCamera()->setProjectionMatrix(m_device->getProjectionMatrixLeft());
		}
		if (m_eye == 1)
		{
			node->asCamera()->setViewMatrix(osg::Matrix::inverse(ref) * m_device->getRightEyeViewMatrix());
			node->asCamera()->setProjectionMatrix(m_device->getProjectionMatrixRight());
		}
		m_device->setControllerMatrixTransform(m_eye, node->asCamera()->getViewMatrix());
		traverse(node, nv);
	}
private:
	int m_eye;
	osg::ref_ptr<OpenVRDevice> m_device;
	std::shared_ptr<osg::Vec3> m_refPos;
	float *m_head;
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