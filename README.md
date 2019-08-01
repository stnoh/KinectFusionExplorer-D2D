# KinectFusionExplorer-D2D
This is the modified version of **"KinectFusionExplorer-D2D"** in Kinect v2 SDK sample code.  
Here are the list of major modifications from original code.  

- CMake-based build chain  
  I removed VS project/solution in the original and replace them by CMakeLists.txt .  
  I recommend to use Visual Studio 2017 or upper version to fully utilize this build tool.  
- add **class KinectFusionProcessorSensor**  
  KinectFusion itself is camera-independent algorithm. However, the original code is quite disappointed because the core part of **KinectFusion** and **depth image sensing from Kinect** are mixed in the **class KinectFusionProcessor** together.  
  From this perspective, I separate the depth sensing part from **KinectFusionProcessor** and transfer this part to **KinectFusionProcessorSensor**.  
- remove unused/redundant code  
  The sample code contains a lot of dummy code. For the readablity and brevity, I removed most of them.  
  However, I still keep "FindCameraPoseAlignDepthFloatToReconstruction()" and "SetReferenceFrame()", which were commented out and was not utilized in the original routine, for further experiment on the camera tracking.  
