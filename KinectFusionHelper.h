//------------------------------------------------------------------------------
// <copyright file="KinectFusionHelper.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "NuiKinectFusionApi.h"

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat);

/// <summary>
/// Test whether the camera moved too far between sequential frames by looking at starting and end transformation matrix.
/// We assume that if the camera moves or rotates beyond a reasonable threshold, that we have lost track.
/// Note that on lower end machines, if the processing frame rate decreases below 30Hz, this limit will potentially have
/// to be increased as frames will be dropped and hence there will be a greater motion between successive frames.
/// </summary>
/// <param name="T_initial">The transform matrix from the previous frame.</param>
/// <param name="T_final">The transform matrix from the current frame.</param>
/// <param name="maxTrans">The maximum translation in meters we expect per x,y,z component between frames under normal motion.</param>
/// <param name="maxRotDegrees">The maximum rotation in degrees we expect about the x,y,z axes between frames under normal motion.</param>
/// <returns>true if camera transformation is greater than the threshold, otherwise false</returns>
bool CameraTransformFailed(const Matrix4 &T_initial, const Matrix4 &T_final, float maxTrans, float maxRotDegrees);

/// <summary>
/// Write Binary .STL mesh file
/// see http://en.wikipedia.org/wiki/STL_(file_format) for STL format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteBinarySTLMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true);

/// <summary>
/// Write ASCII Wavefront .OBJ mesh file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiObjMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true);

/// <summary>
/// Write ASCII .PLY file
/// See http://paulbourke.net/dataformats/ply/ for .PLY format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <param name="outputColor">Set this true to write out the surface color to the file when it has been captured.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiPlyMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true, bool outputColor = false);

/// <summary>
/// Color the residual/delta image from the AlignDepthFloatToReconstruction call
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source pFloatDeltaFromReference image.</param>
/// <param name="pShadedDeltaFromReference">A pointer to the destination ShadedDeltaFromReference image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT ColorResiduals(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference, const NUI_FUSION_IMAGE_FRAME *pShadedDeltaFromReference);

/// <summary>
/// Down sample color, depth float or point cloud frame with nearest neighbor
/// </summary>
/// <param name="src">The source color, depth float or pointcloud image.</param>
/// <param name="dest">The destination down sampled color, depth float or pointcloud image.</param>
/// <param name="factor">The down sample factor (1=just copy, 2=x/2,y/2, 4=x/4,y/4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor);

/// <summary>
/// Up sample color or depth float (32 bits/pixel) frame with nearest neighbor - replicates pixels
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination up-sampled color image.</param>
/// <param name="factor">The up sample factor (1=just copy, 2=x*2,y*2, 4=x*4,y*4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT UpsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor);

/// <summary>
/// Down sample color frame with nearest neighbor to the depth frame resolution
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination down sampled  image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleColorFrameToDepthResolution(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest);

/// <summary>
/// Clamp a value if outside two given thresholds
/// </summary>
/// <param name="x">The value to clamp.</param>
/// <param name="a">The minimum inclusive threshold.</param>
/// <param name="b">The maximum inclusive threshold.</param>
/// <returns>Returns the clamped value.</returns>
template <typename T>
inline T clamp(const T& x, const T& a, const T& b)
{
    if (x < a)
        return a;
    else if (x > b)
        return b;
    else
        return x;
}
