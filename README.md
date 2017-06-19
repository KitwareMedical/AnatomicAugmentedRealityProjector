Ultrasound Augmentation: Rapid 3-D Scanning for Tracking and On-Body Display
============================================================================


By using a laser projector and high speed camera, we can add three capabilities to an ultrasound system: tracking the probe, tracking the patient, and projecting information onto the probe and patient. We can use these capabilities to guide an untrained operator to take high quality, well framed ultrasound images which can be passed to existing or novel diagnosis algorithms.


Requirements :
--------------

* OpenCV 3.1.0
* ITK
* FlyCapture API
* Qt5 

CMake Configuration :
---------------
When configuring this project, be sure to select a 64 bit visual studio generator.
Set Qt5WidgetsDir to (Path to Qt)/Qt/5.9/msvc(your version here)/lib/cmake/Qt5Widgets
Set ITK_DIR to  (Path to ITK)/ITK/build
Be sure ITK is compiled for a 64 bit executable.
Set OpenCV_DIR to (Path to openCV)/opencv/build

Once this project has compiled, copy the missing dlls from their respective bin folders.
