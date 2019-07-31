//------------------------------------------------------------------------------
// This code is separated from original "KinectFusionProcessor.cpp"
// It only handles to copy color/depth images and undistort depth image.
// modified by Seung-Tak Noh (seungtak.noh@gmail.com)
//------------------------------------------------------------------------------

// System includes
#include "stdafx.h"

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

// Project includes
#include "KinectFusionProcessorSensor.h"

/// <summary>
/// Copy and do depth frame undistortion.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessorSensor::CopyDepth(INT64& currentDepthFrameTime)
{
	// wrapping previous function by lambda
	auto copyDepth = [&](IDepthFrame* pDepthFrame)->HRESULT {
		// Check the frame pointer
		if (NULL == pDepthFrame)
		{
			return E_INVALIDARG;
		}

		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		HRESULT hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		if (FAILED(hr))
		{
			return hr;
		}

		//copy and remap depth
		const UINT bufferLength =  NUI_DEPTH_RAW_HEIGHT * NUI_DEPTH_RAW_WIDTH;
		UINT16 * pDepth = m_pDepthUndistortedPixelBuffer;
		UINT16 * pRawDepth = m_pDepthRawPixelBuffer;
		for(UINT i = 0; i < bufferLength; i++, pDepth++, pRawDepth++)
		{
			const UINT id = m_pDepthDistortionLT[i];
			*pDepth = id < bufferLength? pBuffer[id] : 0;
			*pRawDepth = pBuffer[i];
		}

		return S_OK;
	};

	HRESULT hr = S_OK;

	IDepthFrame* pDepthFrame = NULL;
	hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (FAILED(hr))
	{
		SafeRelease(pDepthFrame);
		//SetStatusMessage(L"Kinect depth stream get frame call failed."); // no message box in this context ...
		return hr;
	}

	copyDepth(pDepthFrame);
	pDepthFrame->get_RelativeTime(&currentDepthFrameTime);
	currentDepthFrameTime /= 10000;

	SafeRelease(pDepthFrame);

	return S_OK;
}

/// <summary>
/// Shuts down the sensor
/// </summary>
void KinectFusionProcessorSensor::ShutdownSensor()
{
	// Clean up Kinect
	if (m_pNuiSensor != nullptr)
	{
		m_pNuiSensor->Close();
		SafeRelease(m_pNuiSensor);
	}
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT KinectFusionProcessorSensor::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pNuiSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pNuiSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;
		IColorFrameSource* pColorFrameSource = NULL;

		hr = m_pNuiSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pNuiSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pNuiSensor->get_CoordinateMapper(&m_pMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pNuiSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		SafeRelease(pDepthFrameSource);
		SafeRelease(pColorFrameSource);
	}

	if (!m_pNuiSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!"); // no message box in this context ...
		return E_FAIL;
	}

	return hr;
}

/// <summary>
/// Setup or update the Undistortion calculation for the connected camera
/// </summary>
HRESULT KinectFusionProcessorSensor::SetupUndistortion(NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters, UINT width, UINT height, bool& m_bHaveValidCameraParameters)
{
    HRESULT hr = E_UNEXPECTED;

    if (m_cameraParameters.principalPointX != 0)
    {
        const UINT depthBufferSize = width * height;

        CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
        {
            /*LT*/ { -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f }, 
            /*RT*/ { (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f }, 
            /*LB*/ { -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }, 
            /*RB*/ { (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
        };

        for(UINT rowID = 0; rowID < height; rowID++)
        {
            const float rowFactor = float(rowID) / float(height - 1);
            const CameraSpacePoint rowStart = 
            {
                cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
                cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
                1.f
            };

            const CameraSpacePoint rowEnd = 
            {
                cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
                cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
                1.f
            };

            const float stepFactor = 1.f / float(width - 1);
            const CameraSpacePoint rowDelta = 
            {
                (rowEnd.X - rowStart.X) * stepFactor,
                (rowEnd.Y - rowStart.Y) * stepFactor,
                0
            };

            _ASSERT(width == NUI_DEPTH_RAW_WIDTH);
            CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

            CameraSpacePoint currentPoint = rowStart;
            for(UINT i = 0; i < width; i++)
            {
                cameraCoordsRow[i] = currentPoint;
                currentPoint.X += rowDelta.X;
                currentPoint.Y += rowDelta.Y;
            }

            hr = m_pMapper->MapCameraPointsToDepthSpace(width, cameraCoordsRow, width, &m_pDepthDistortionMap[rowID * width]);
            if(FAILED(hr))
            {
                //SetStatusMessage(L"Failed to initialize Kinect Coordinate Mapper."); // no message box in this context ...
                return hr;
            }
        }

        if (nullptr == m_pDepthDistortionLT)
        {
            //SetStatusMessage(L"Failed to initialize Kinect Fusion depth image distortion Lookup Table."); // no message box in this context ...
            return E_OUTOFMEMORY;
        }

        UINT* pLT = m_pDepthDistortionLT;
        for(UINT i = 0; i < depthBufferSize; i++, pLT++)
        {
            //nearest neighbor depth lookup table 
            UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
            UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

            *pLT = (x < width && y < height)? x + y * width : UINT_MAX; 
        } 
        m_bHaveValidCameraParameters = true;
    }
    else
    {
        m_bHaveValidCameraParameters = false;
    }
    return S_OK;
}

/// <summary>
/// Get Color data
/// </summary>
/// <param name="imageFrame">The color image frame to copy.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectFusionProcessorSensor::CopyColor(const NUI_FUSION_IMAGE_FRAME* m_pColorImage, INT64& currentColorFrameTime, bool& colorSynchronized)
{
	// wrapping previous function by lambda
	auto copyColor = [&](IColorFrame* pColorFrame)->HRESULT {
		HRESULT hr = S_OK;

		if (nullptr == m_pColorImage)
		{
			//SetStatusMessage(L"Error copying color texture pixels."); // no message box in this context ...
			return E_FAIL;
		}

		NUI_FUSION_BUFFER *destColorBuffer = m_pColorImage->pFrameBuffer;

		if (nullptr == pColorFrame || nullptr == destColorBuffer)
		{
			return E_NOINTERFACE;
		}

		// Copy the color pixels so we can return the image frame
		hr = pColorFrame->CopyConvertedFrameDataToArray(m_pColorImage->width * m_pColorImage->height * sizeof(RGBQUAD), destColorBuffer->pBits, ColorImageFormat_Bgra);

		if (FAILED(hr))
		{
			//SetStatusMessage(L"Error copying color texture pixels."); // no message box in this context ...
			hr = E_FAIL;
		}

		return hr;
	};

	HRESULT hr = S_OK;

	IColorFrame* pColorFrame = NULL;
	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (FAILED(hr))
	{
		// Here we just do not integrate color rather than reporting an error
		colorSynchronized = false;
	}
	else
	{
		if (SUCCEEDED(hr))
		{
			copyColor(pColorFrame);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RelativeTime(&currentColorFrameTime);
			currentColorFrameTime /= 10000;
		}
	}
	SafeRelease(pColorFrame);

	return S_OK;
}

/// <summary>
/// Adjust color to the same space as depth
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
HRESULT KinectFusionProcessorSensor::MapColorToDepth(const NUI_FUSION_IMAGE_FRAME* m_pColorImage, NUI_FUSION_IMAGE_FRAME* m_pResampledColorImageDepthAligned, KinectFusionParams m_paramsCurrent)
{
    HRESULT hr = S_OK;

    if (nullptr == m_pColorImage || nullptr == m_pResampledColorImageDepthAligned 
		|| nullptr == m_pColorCoordinates || nullptr == m_pDepthVisibilityTestMap)
	{
        return E_FAIL;
    }

    NUI_FUSION_BUFFER *srcColorBuffer = m_pColorImage->pFrameBuffer;
    NUI_FUSION_BUFFER *destColorBuffer = m_pResampledColorImageDepthAligned->pFrameBuffer;

    if (nullptr == srcColorBuffer || nullptr == destColorBuffer)
    {
        //SetStatusMessage(L"Error accessing color textures."); // no message box in this context ...
        return E_NOINTERFACE;
    }

    if (FAILED(hr) || srcColorBuffer->Pitch == 0)
    {
        //SetStatusMessage(L"Error accessing color texture pixels."); // no message box in this context ...
        return  E_FAIL;
    }

    if (FAILED(hr) || destColorBuffer->Pitch == 0)
    {
        //SetStatusMessage(L"Error accessing color texture pixels."); // no message box in this context ...
        return  E_FAIL;
    }

    int *rawColorData = reinterpret_cast<int*>(srcColorBuffer->pBits);
    int *colorDataInDepthFrame = reinterpret_cast<int*>(destColorBuffer->pBits);

    // Get the coordinates to convert color to depth space
    hr = m_pMapper->MapDepthFrameToColorSpace(NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pDepthRawPixelBuffer, 
        NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pColorCoordinates);

    if (FAILED(hr))
    {
        return hr;
    }

	int cVisibilityTestQuantShift = m_paramsCurrent.m_cAlignPointCloudsImageDownsampleFactor;

    // construct dense depth points visibility test map so we can test for depth points that are invisible in color space
    const UINT16* const pDepthEnd = m_pDepthRawPixelBuffer + NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT;
    const ColorSpacePoint* pColorPoint = m_pColorCoordinates;
    const UINT testMapWidth  = UINT(m_pColorImage->width  >> cVisibilityTestQuantShift);
    const UINT testMapHeight = UINT(m_pColorImage->height >> cVisibilityTestQuantShift);
    ZeroMemory(m_pDepthVisibilityTestMap, testMapWidth * testMapHeight * sizeof(UINT16));
    for(const UINT16* pDepth = m_pDepthRawPixelBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
    {
        const UINT x = UINT(pColorPoint->X + 0.5f) >> cVisibilityTestQuantShift;
        const UINT y = UINT(pColorPoint->Y + 0.5f) >> cVisibilityTestQuantShift;
        if(x < testMapWidth && y < testMapHeight)
        {
            const UINT idx = y * testMapWidth + x;
            const UINT16 oldDepth = m_pDepthVisibilityTestMap[idx];
            const UINT16 newDepth = *pDepth;
            if(!oldDepth || oldDepth > newDepth)
            {
                m_pDepthVisibilityTestMap[idx] = newDepth;
            }
        }
    }


    // Loop over each row and column of the destination color image and copy from the source image
    // Note that we could also do this the other way, and convert the depth pixels into the color space, 
    // avoiding black areas in the converted color image and repeated color images in the background
    // However, then the depth would have radial and tangential distortion like the color camera image,
    // which is not ideal for Kinect Fusion reconstruction.

    if (m_paramsCurrent.m_bMirrorDepthFrame)
    {
        Concurrency::parallel_for(0u, m_paramsCurrent.m_cDepthHeight, [&](UINT y)
        {
            const UINT depthWidth = m_paramsCurrent.m_cDepthWidth;
            const UINT depthImagePixels = m_paramsCurrent.m_cDepthImagePixels;
            const UINT colorHeight = m_paramsCurrent.m_cColorHeight;
            const UINT colorWidth = m_paramsCurrent.m_cColorWidth;
            const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

            UINT destIndex = y * depthWidth;
            for (UINT x = 0; x < depthWidth; ++x, ++destIndex)
            {
                int pixelColor = 0;
                const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
                if(mappedIndex < depthImagePixels)
                {
                    // retrieve the depth to color mapping for the current depth pixel
                    const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

                    // make sure the depth pixel maps to a valid point in color space
                    const UINT colorX = (UINT)(colorPoint.X + 0.5f);
                    const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
                    if (colorX < colorWidth && colorY < colorHeight)
                    {
                        const UINT16 depthValue = m_pDepthRawPixelBuffer[mappedIndex];
                        const UINT testX = colorX >> cVisibilityTestQuantShift;
                        const UINT testY = colorY >> cVisibilityTestQuantShift;
                        const UINT testIdx = testY * testMapWidth + testX;
                        const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
                        _ASSERT(depthValue >= depthTestValue);
                        if(depthValue - depthTestValue < cDepthVisibilityTestThreshold)
                        {
                            // calculate index into color array
                            const UINT colorIndex = colorX + (colorY * colorWidth);
                            pixelColor = rawColorData[colorIndex];
                        }
                    }
                }
                colorDataInDepthFrame[destIndex] = pixelColor;
            }
        });
    }
    else
    {
        Concurrency::parallel_for(0u, m_paramsCurrent.m_cDepthHeight, [&](UINT y)
        {
            const UINT depthWidth = m_paramsCurrent.m_cDepthWidth;
            const UINT depthImagePixels = m_paramsCurrent.m_cDepthImagePixels;
            const UINT colorHeight = m_paramsCurrent.m_cColorHeight;
            const UINT colorWidth = m_paramsCurrent.m_cColorWidth;
            const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

            // Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
            // to give a viewpoint as though from behind the Kinect looking forward by default.
            UINT destIndex = y * depthWidth;
            UINT flipIndex = destIndex + depthWidth - 1;
            for (UINT x = 0; x < depthWidth; ++x, ++destIndex, --flipIndex)
            {
                int pixelColor = 0;
                const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
                if(mappedIndex < depthImagePixels)
                {
                    // retrieve the depth to color mapping for the current depth pixel
                    const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

                    // make sure the depth pixel maps to a valid point in color space
                    const UINT colorX = (UINT)(colorPoint.X + 0.5f);
                    const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
                    if (colorX < colorWidth && colorY < colorHeight)
                    {
                        const UINT16 depthValue = m_pDepthRawPixelBuffer[mappedIndex];
                        const UINT testX = colorX >> cVisibilityTestQuantShift;
                        const UINT testY = colorY >> cVisibilityTestQuantShift;
                        const UINT testIdx = testY * testMapWidth + testX;
                        const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
                        _ASSERT(depthValue >= depthTestValue);
                        if(depthValue - depthTestValue < cDepthVisibilityTestThreshold)
                        {
                            // calculate index into color array
                            const UINT colorIndex = colorX + (colorY * colorWidth);
                            pixelColor = rawColorData[colorIndex];
                        }
                    }
                }
                colorDataInDepthFrame[flipIndex] = pixelColor;
            }
        });
    }

    return hr;
}


/// <summary>
/// Constructor
/// </summary>
KinectFusionProcessorSensor::KinectFusionProcessorSensor() :
	m_pNuiSensor(nullptr)
	, m_pDepthUndistortedPixelBuffer(nullptr)
	, m_pDepthRawPixelBuffer(nullptr)
	, m_pColorCoordinates(nullptr)
	, m_pDepthVisibilityTestMap(nullptr)
	, m_pMapper(nullptr)
	, m_pDepthDistortionMap(nullptr)
	, m_pDepthDistortionLT(nullptr)
{
	// [TODO?]
}

/// <summary>
/// Destructor
/// </summary>
KinectFusionProcessorSensor::~KinectFusionProcessorSensor()
{
	SafeRelease(m_pMapper);

	// Clean up the depth pixel array
	SAFE_DELETE_ARRAY(m_pDepthUndistortedPixelBuffer);
	SAFE_DELETE_ARRAY(m_pDepthRawPixelBuffer);

	// Clean up the color coordinate array
	SAFE_DELETE_ARRAY(m_pColorCoordinates);
	SAFE_DELETE_ARRAY(m_pDepthVisibilityTestMap);

	SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
	SAFE_DELETE_ARRAY(m_pDepthDistortionLT);

	// done with depth frame reader
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pColorFrameReader);
}
