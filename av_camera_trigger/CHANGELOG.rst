^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_camera_trigger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-11-26)
------------------
* Set frame rate to 20 fps
 - Previously, the trigger was set to 40 fps, but it was later discovered that 
   the camera was configured to the wrong triggering GPIO, causing the camera FPS to 
   operate at half the defined value. The updated ROS camera driver now ensures 
   the camera FPS is consistent with the trigger frame rate, removing the need to 
   compensate by setting the frame rate to double the desired triggering FPS.

* Contributors: hect95


1.1.0 (2024-11-20)
------------------
* Fix host - trigger device time synchronisation (`#3 <https://github.com/ipab-rad/av_camera_trigger/issues/3>`_)
  -Pass ctime in seconds to trigger device rather than rostime nanoseconds
* Contributors: Hector Cruz

1.0.0 (2024-07-05)
------------------
* Update cams to run at 20Hz (40 / 2)
* Port camera trigger code from legacy repo
* Contributors: Alejandro Bordallo, hect95
