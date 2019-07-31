//------------------------------------------------------------------------------
// This code is separated from original "KinectFusionProcessor.h"
// It only handles to copy color/depth images and undistort depth image.
// modified by Seung-Tak Noh (seungtak.noh@gmail.com)
//------------------------------------------------------------------------------

#pragma once

#include <vector>
#include "NuiKinectFusionApi.h"

#include "KinectFusionParams.h"

/// <summary>
/// Kinect controlling part in Kinect Fusion algorithm.
/// </summary>
class KinectFusionProcessorSensor
{
	static const UINT16         cDepthVisibilityTestThreshold = 50; //50 mm

public:

	/// <summary>
	/// Constructor
	/// </summary>
	KinectFusionProcessorSensor();

	/// <summary>
	/// Destructor
	/// </summary>
	~KinectFusionProcessorSensor();

	/// <summary>
	/// Initialize the kinect sensor.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     InitializeDefaultSensor();

	/// <summary>
	/// Shuts down the sensor.
	/// </summary>
	void                        ShutdownSensor();

	IKinectSensor*              m_pNuiSensor;
	IDepthFrameReader*          m_pDepthFrameReader;
	IColorFrameReader*          m_pColorFrameReader;

	/// <summary>
	/// Setup or update the Undistortion calculation for the connected camera
	/// </summary>
	HRESULT                     SetupUndistortion(NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters, UINT width, UINT height, bool& m_bHaveValidCameraParameters);

	/// <summary>
	/// Copy and do depth frame undistortion.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CopyDepth(INT64& currentDepthFrameTime);

	/// <summary>
	/// Copy the color data out of a Kinect image frame
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     CopyColor(const NUI_FUSION_IMAGE_FRAME* m_pColorImage, INT64& currentColorFrameTime, bool& colorSynchronized);

	/// <summary>
	/// Adjust color to the same space as depth
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     MapColorToDepth(const NUI_FUSION_IMAGE_FRAME* m_pColorImage, NUI_FUSION_IMAGE_FRAME* m_pResampledColorImageDepthAligned, KinectFusionParams m_paramsCurrent);

	/// <summary>
	/// Frames from the depth input.
	/// </summary>
	UINT16*                     m_pDepthUndistortedPixelBuffer;
	UINT16*                     m_pDepthRawPixelBuffer;

	/// <summary>
	/// For mapping color to depth and depth distortion correction
	/// </summary>
	ColorSpacePoint*            m_pColorCoordinates;
	UINT16*                     m_pDepthVisibilityTestMap;
	ICoordinateMapper*          m_pMapper;

	DepthSpacePoint*            m_pDepthDistortionMap;
	UINT*                       m_pDepthDistortionLT;
};
