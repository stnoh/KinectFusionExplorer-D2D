﻿//------------------------------------------------------------------------------
// <copyright file="KinectFusionProcessor.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <vector>
#include "NuiKinectFusionApi.h"

#include "Timer.h"
#include "KinectFusionParams.h"
#include "KinectFusionProcessorFrame.h"

#include "KinectFusionHelper.h"
#include "KinectFusionProcessorSensor.h"


/// <summary>
/// Performs all Kinect Fusion processing for the KinectFusionExplorer.
/// All data capture and processing is done on a worker thread.
/// </summary>
class KinectFusionProcessor
{
    static const int            cResetOnTimeStampSkippedMillisecondsGPU = 2000;
    static const int            cResetOnTimeStampSkippedMillisecondsCPU = 6000;
    static const int            cResetOnNumberOfLostFrames = 100;
    static const int            cTimeDisplayInterval = 4;
    static const int            cRenderIntervalMilliseconds = 100; // Render every 100ms
    static const int            cMinTimestampDifferenceForFrameReSync = 30; // The minimum timestamp difference between depth and color (in ms) at which they are considered un-synchronized.

public:

    /// <summary>
    /// Constructor
    /// </summary>
    KinectFusionProcessor();

    /// <summary>
    /// Destructor
    /// </summary>
    ~KinectFusionProcessor();

    /// <summary>
    /// Sets the UI window handle.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     SetWindow(HWND hWnd, UINT msgFrameReady, UINT msgUpdateSensorStatus);

    /// <summary>
    /// Sets the parameters.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     SetParams(const KinectFusionParams& params);

    /// <summary>
    /// Starts Kinect Fusion processing.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     StartProcessing();

    /// <summary>
    /// Stops Kinect Fusion processing.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     StopProcessing();

    /// <summary>
    /// Reset the reconstruction camera pose and clear the volume on the next frame.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     ResetReconstruction();

    /// <summary>
    /// Calculate a mesh for the current volume.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     CalculateMesh(INuiFusionColorMesh** ppMesh);

    /// <summary>
    /// Lock the current frame while rendering it to the screen.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     LockFrame(KinectFusionProcessorFrame const** ppFrame);

    /// <summary>
    /// Unlock the previously locked frame.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     UnlockFrame();

    /// <summary>
    /// Is reconstruction volume initialized and running.
    /// </summary>
    bool                        IsVolumeInitialized();

    /// <summary>
    /// Is the camera pose finder initialized and running.
    /// </summary>
    bool                        IsCameraPoseFinderAvailable();

private:
    KinectFusionParams          m_paramsNext;
    KinectFusionParams          m_paramsCurrent;
    CRITICAL_SECTION            m_lockParams;

    HWND                        m_hWnd;
    UINT                        m_msgFrameReady;
    UINT                        m_msgUpdateSensorStatus;

    HANDLE                      m_hThread;
    DWORD                       m_threadId;

	KinectFusionProcessorSensor m_pSensor; // facade class for Kinect as RAII object

    LONGLONG                    m_cLastDepthFrameTimeStamp;
    LONGLONG                    m_cLastColorFrameTimeStamp;

    WCHAR                       m_statusMessage[KinectFusionProcessorFrame::StatusMessageMaxLen];

    KinectFusionProcessorFrame  m_frame;
    CRITICAL_SECTION            m_lockFrame;

    /// <summary>
    /// Thread procedure.
    /// </summary>
    static DWORD WINAPI         ThreadProc(LPVOID lpParameter);

    /// <summary>
    /// Main processing function
    /// </summary>
    DWORD                       MainLoop();

    /// <summary>
    /// Handle Coordinate Mapping changed event.
    /// Note, this happens after sensor connect, or when Kinect Studio connects
    /// </summary>
    HRESULT                     OnCoordinateMappingChanged();

    /// <summary>
    /// Initialize Kinect Fusion volume and images for processing
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     InitializeKinectFusion();

    /// <summary>
    /// Create a Kinect Fusion image frame of the specified type.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     CreateFrame(
                                    NUI_FUSION_IMAGE_TYPE frameType,
                                    unsigned int imageWidth,
                                    unsigned int imageHeight,
                                    NUI_FUSION_IMAGE_FRAME** ppImageFrame);

    /// <summary>
    /// Release and re-create a Kinect Fusion Reconstruction Volume.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     RecreateVolume();

    /// <summary>
    /// Get the next frames from Kinect, re-synchronizing depth with color if required.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     GetKinectFrames(bool &integrateColor);

    /// <summary>
    /// Handle new depth data and perform Kinect Fusion Processing.
    /// </summary>
    bool                        IntegrateSingleFrame();

    /// <summary>
    /// Perform camera tracking using AlignDepthFloatToReconstruction
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     TrackCameraAlignDepthFloatToReconstruction(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);

    /// <summary>
    /// Perform camera tracking using AlignPointClouds
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     TrackCameraAlignPointClouds(Matrix4 &calculatedCameraPose, FLOAT &alignmentEnergy);

    /// <summary>
    /// Perform camera pose finding when tracking is lost using AlignPointClouds.
    /// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     FindCameraPoseAlignPointClouds();

    /// <summary>
    /// Perform camera pose finding when tracking is lost using AlignDepthFloatToReconstruction.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     FindCameraPoseAlignDepthFloatToReconstruction();

    /// <summary>
    /// Performs raycasting for given pose and sets the tracking reference frame.
    /// </summary>
    /// <param name="worldToCamera">The reference camera pose.</param>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     SetReferenceFrame(const Matrix4 &worldToCamera);

    /// <summary>
    /// Set the tracking variables to tracking failed.
    /// </summary>
    void                        SetTrackingFailed();

    /// <summary>
    /// Set the tracking variables to tracking success.
    /// </summary>
    void                        SetTrackingSucceeded();

    /// <summary>
    /// Reset the tracking following a reset or re-create of the volume.
    /// </summary>
    void                        ResetTracking();

    /// <summary>
    /// Update data for camera pose finding and store key frame poses.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     UpdateCameraPoseFinder();

    /// <summary>
    /// Store a Kinect Fusion image to a frame buffer.
    /// Accepts Depth Float, and Color image types.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     StoreImageToFrameBuffer(
                                    const NUI_FUSION_IMAGE_FRAME* imageFrame,
                                    BYTE* buffer);

    /// <summary>
    /// Reset the reconstruction camera pose and clear the volume.
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                     InternalResetReconstruction();

    /// <summary>
    /// Set the status bar message.
    /// </summary>
    /// <param name="szMessage">message to display</param>
    void                        SetStatusMessage(WCHAR* szMessage);

    void                        NotifyFrameReady();
    void                        NotifyEmptyFrame();

    bool                        m_bKinectFusionInitialized;
    bool                        m_bHaveValidCameraParameters;
    bool                        m_bResetReconstruction;
    bool                        m_bResolveSensorConflict;

    /// <summary>
    /// Used for informing of stopping processing thread.
    /// </summary>
    HANDLE                      m_hStopProcessingEvent;

    /// <summary>
    /// The Kinect Fusion Volume.
    /// </summary>
    INuiFusionColorReconstruction* m_pVolume;
    HRESULT                     m_hrRecreateVolume;
    CRITICAL_SECTION            m_lockVolume;

    /// <summary>
    // The Kinect Fusion Camera Transform.
    /// </summary>
    Matrix4                     m_worldToCameraTransform;

    /// <summary>
    // The default Kinect Fusion World to Volume Transform.
    /// </summary>
    Matrix4                     m_defaultWorldToVolumeTransform;

    /// <summary>
    /// Kinect camera parameters.
    /// </summary>
    NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters; 

    /// <summary>
    /// Frames generated from the depth input for AlignPointClouds
    /// </summary>
    NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthFloatImage;
    NUI_FUSION_IMAGE_FRAME*     m_pDownsampledSmoothDepthFloatImage;
    NUI_FUSION_IMAGE_FRAME*     m_pDownsampledDepthPointCloud;

    /// <summary>
    /// For mapping color to depth and depth distortion correction
    /// </summary>
    NUI_FUSION_IMAGE_FRAME*     m_pColorImage;
    NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImageDepthAligned;
	WAITABLE_HANDLE             m_coordinateMappingChangedEvent;

    /// <summary>
    /// Frames generated from ray-casting the Reconstruction Volume.
    /// </summary>
    Matrix4                     m_worldToBGRTransform;
    NUI_FUSION_IMAGE_FRAME*     m_pRaycastPointCloud;
    NUI_FUSION_IMAGE_FRAME*     m_pRaycastDepthFloatImage;
    NUI_FUSION_IMAGE_FRAME*     m_pDownsampledRaycastPointCloud;

    /// <summary>
    /// Images for display.
    /// </summary>
    NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;
    NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;
    NUI_FUSION_IMAGE_FRAME*     m_pShadedSurfaceNormals;
    NUI_FUSION_IMAGE_FRAME*     m_pCapturedSurfaceColor;
    NUI_FUSION_IMAGE_FRAME*     m_pFloatDeltaFromReference;
    NUI_FUSION_IMAGE_FRAME*     m_pShadedDeltaFromReference;
    NUI_FUSION_IMAGE_FRAME*     m_pDownsampledShadedDeltaFromReference;

    /// <summary>
    /// Camera Tracking parameters.
    /// </summary>
    int                         m_cLostFrameCounter;
    bool                        m_bTrackingFailed;

    /// <summary>
    /// Camera Pose Finder.
    /// Note color will be re-sampled to the depth size if depth and color capture resolutions differ.
    /// </summary>
    INuiFusionCameraPoseFinder* m_pCameraPoseFinder;
    NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImage;
    NUI_FUSION_IMAGE_FRAME*     m_pDepthPointCloud;
    NUI_FUSION_IMAGE_FRAME*     m_pSmoothDepthFloatImage;
    unsigned                    m_cSuccessfulFrameCounter;
    bool                        m_bTrackingHasFailedPreviously;
    bool                        m_bCalculateDeltaFrame;

    /// <summary>
    /// Frame counter and timer.
    /// </summary>
    int                         m_cFrameCounter;
    int                         m_cFPSFrameCounter;
    Timing::Timer               m_timer;
    double                      m_fFrameCounterStartTime;
    double                      m_fMostRecentRaycastTime;

	////////////////////////////////////////////////////////////
	// additional members to use KinectFusionHelperEx
	////////////////////////////////////////////////////////////
public:
	HRESULT Load(const std::string& filename);
	HRESULT Save(const std::string& filename);

private:
	void InternalLoad();

	bool m_internal_load = false;

	Matrix4 _worldToCameraTransform;
	Matrix4 _worldToVolumeTransform;
	std::vector<short> _volumeBlock;
	std::vector<int>   _colorVolumeBlock;
};
