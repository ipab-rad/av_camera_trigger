^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_camera_trigger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
