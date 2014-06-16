^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package msf_updates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* minor fixes
* Fixed compiler errors
* adr comments
* Merge remote-tracking branch 'origin/master' into feature/pose_measurement_templated
  Conflicts:
  msf_updates/include/msf_updates/pose_sensor_handler/implementation/pose_sensorhandler.hpp
  msf_updates/include/msf_updates/pose_sensor_handler/pose_measurement.h
* minor style
* more style changes
* Merged origin/master and changed new code to style
* Changes for compatibility with gcc4.6 on the build server
* templated pose measurement on its auxiliary state
  camera-imu calibration, world-vision frame and scale are now template parameters, such that we can use the pose measurement implementation for multiple measurements, incorporating the correct aux states.
  templates have default parameters, therefore the current implementation shouldnt be affected.
* linebreak
* added check for abs(height) > 0
* comments
* added init-services (height & scale) for pose_sensor
* Minor changes concerning core parameter output. Adapted vicon launch- and parameter file accordingly.
* adr comments
* additional style fixes
* cleanup for merging
* when compiling with debug info, compiler didn't know != operator for
  iterator ?!?!
  added debug info to msf_updates
* simon's comments
* added apache license to packag.xml files
* touch
* added more descriptive comments to the statedef
* Update position_pose_sensormanager.h
* More descriptive core state comments
* Added more descriptive comments to the state definition
* compile flag cleanup; msf_updates compile as release now; dynamic_reconfigure dependencies fixed-may require latest catkin version
* initial catkinization
* markus comments, made more functions pure virtual in sensor manager base and const corrected spherical sensor to work with current master
* touch
* Cleanup linting of spherical sensor.
* moving to correct folders
* Adding spherical msf version of georg.
* linted msfupdates
* apache 2.0
* linted msf_core
* simulator draft
* Logging macros for msf
* added state callback buffering for hl correction
* moved publisher to ROS sensorhandler and moved imu subscriber to ros imu
  handler
* added posefilter ignore option
* made shared pointer type variable
* Wno_unused_parameter
* std::shared_ptr
* added aslam params
* minor
* Revert "Merge branch 'spherical_sensor' of github.com:ethz-asl/sensor_fusion"
  This reverts commit a894d48c9ab8c7f6e64c7a85d8a503a791242526, reversing
  changes made to b3178d55e586b79ceecdf2d8869219aefc47127d.
* Merge branch 'spherical_sensor' of github.com:ethz-asl/sensor_fusion
* M_PI instead of 3.14...
* crossing of x-axis solved in residual calculation
* change params
* added fixparams to parameter files
* lint formatting
* const corrected msf_sensor manager state visitor
* fix compile error state visitor
* inserted residuals - everything fine now :)
* spherical launch and parameters
* error - unsolved
* minor changes for function name printing
* rebased master
* completed spherical H-Matrix
* restore some files I removed by accident
* getting started on the spherical H matrix
* spherical sensor uses fixed cov by default now
* fixed measurement
* adapted to update in measurement base class
* added skeleton for spherical position sensor
* completed spherical H-Matrix
* disable scale corr mult, not working right now
* restore some files I removed by accident
* getting started on the spherical H matrix
* spherical sensor uses fixed cov by default now
* fixed measurement
* adapted to update in measurement base class
* Merge branch 'master' into spherical_sensor
* undid some stupid mistake ;)
* replaced -std=c++0x by -std=c++11
* added switch for -march=native because my mac compiler is too stupid for this
* added skeleton for spherical position sensor
* adapted the messages to the new measurement base class with template T
  Matrix type
* boost::shared_dynamic_cast is deprecated and apparently removed for
  boost 1.53 in combination with c++11 --> use boost_dynamic_pointer_cast
  now
* added options to statevar
* added msg dropout watchdogs and increase queuesizes. can process 750Hz
  IMU, 450Hz vision and 150Hz GPS without dropouts.
* added gps and changed pointwCov
* rm indices in statedef for non core states
* position-pose filter working
* the position-pose sensor still needs debugging for the initial values.
* compiles -Wall -Wextra -pedantic without warnings
* pose_position position_offset ok
* removed some more superflux transpose of conjugate from the
  pose_measurements implementation
* changed imu->cam calibration indices to fit the new imu-centered
  notation. changed world->vision drift to fit the new world-centered
  notation
* changed position world vision offset to correct notation and adjusted
  indices
* removed all -O2 from child directories, so we can control the build type
  from the top dir. Ros Release build types are anyway O2
* fixed q_vw pose sensor bug
* removed results files
* debugging pose position
* renamed position_vision to position_pose and added launch
* more parameter changes
* reorganized all parameters
* leica position filter fix calib
* cmake list reflect move gpsvision
* friend decl gps_vision move gps_vision
* friend decl posepress
* decl friend class
* decl friend
* vision-gps sensor draft
* eval
* minor
* added fix vw drifts
* added warning on no meas applied
* added march native
* readded distort in Cmake
* Merge branch 'master_eval' into sc
  Conflicts:
  msf_updates/src/pose_msf/pose_sensormanager.h
* removed posedist libs
* added clear cross cov on state fix, will do pseudo meas later as a
  better solution
* added cov image publisher
* minor
* removed vicon-meas and changed the fix state calculation
* added a dedicated vicon-pose filter variant without superflux
  calibration states
* changed the pose sensor manager to be a template for pose measurement
  type
* added pose distorter
* SC seems to be working, needs comparison with absolute meas and some
  optimization in apply-corr-relative
* evalparams
* eval
* params
* imu cam params
* changed friend decl
* ext EKF fix bug no publish on state nan check
* removed friend typedef
* msf_position sensor seems to work, tested with vicon-position only data
* added position msf_ compiles, untested
* added preparations for SC. Mainly sensor IDs for measurements
* added exports to msf_updates
* moved state echo after init to core
* added gitignore for cfg subdir
* readded corrected Wno-string-compare
* removed asctec mav imu msgs inclusion
* minor reorganized header inclusions, removed ros calls in subdir cmake
  files
* init pose sensor handler to zero/unit, minor changes to remove warnings
* Added warning if message throttling is on
* fixed the error that the hl-state messages where overwriting the
  covariance of the init state if the after the init there was another
  message arriving from the hl with the old state values.
* fixed timing issues when used with ext propagation. Now working with
  pressure sensor and height init.
* changed correction index calculation to a new simpler to use meta
  programming struct
* changes to the pose pressure sensor, which is not working at the moment.
* moved pvw next to qvw in the state definition and adapted the plotting scripts
* changed indexing in pose measurement to use compile time calculated indices
* added vision-world position drift state
* added a check for positive definite covariance matrix in pose sensor
* adding aslam config files, set frameid of published pose to /world
* minor change to saveguard the init procedure from other
  threads applying measurements
* added additional checks for EKF reset in the add-measurements method
* removed Const typedefs and fixed affected files
* added additional safety checks for numeric problems, still with some init requests the initial state is containing some NaNs
* added measurement queue for cases where the propagation is not ready when a measurement arrives
* changed O3 -> O2. Enough for now, compiling faster with less mem usage
* removed bug in sortedContainer to check for value at time if closest to time is requested. changed pressure sensor to queue measurements
* removed bug in pose pressure sensor, wrong sign of measurement matrix entry for pressure sensor. Still the delay compensation has some problems, spiking
* changed H matrix calculation for pressure sensor
* added height init, pressure averaging for pressure sensor and additional
  getter for this value.
* added pose pressure cfg
* changed header file extensions to match ROS code style
* fixed a bug in the multi-sensor filter with state interpolation and
  creation of new states at the time of the measurement. this might also
  have fixed the multi-sensor filter fuzzy tracking, but not tested
* another try for the pressure sensor
* modified pressure sensor, but still erroneous
* pressure sensor not working, fixed propagation infinite loop in applymeas of core
* removed eigen matrix access errors in pressure measurement
* changed enum values of the state and renamed state member variables.
  Finished impl of pressure-pose sensor and refactored the header
  organization of msf_statedef
* added static assertion checker for core state ordering in separate file, core is now completely free of state definitions
* moved state definition from core to msf_updates subdirs to be defined for every single sensor
* templated measurements with EKFState_T
* templated core classes to EKFState type
* changed namespace of statedef to msf_updates
* changed formatting to ros style file
* Merge branch 'modular_sensor_fusion' of github.com:ethz-asl/sensor_fusion into modular_sensor_fusion
* fixed non-initialized variables
* changed default value for delay
* added markus to copy header
* removed generated config files
* added cfg file for pose sensor
* dynamic reconfigure reoorganized, delay moved to measurement, works with tutorial dataset
* changed meas covariance bug, which was not setting the member variable, minor cleanup
* fixed swapped usefixed_covariance and measurement world_sensor
* initial version of a pose sensor handler, with overloads for tf, pose, posewcov
* modified ext ekf size and working on pose sensor, but not finished
* added doxygen config and layout file and continued on the documentation
* added documentation to all methods/classes/variables
* added option to use simulated core covariance and fixed diag. for aux states. Alternatively the user can still provide a full simulated cov. matrix as before
* changed stack descr. and removed some small parts in the code which were unused
* msf working with vicon dataset. Giving the exact same results as ssf with the new infinite statebuffer
* still crashing with some init values. It seems that the selection of states for the update is not done correctly sometimes
* added liscense to all files. Debugging stateMap and measMap. Prediction ok, update not working so far
* fixed initialization to use a measurement object and removed the bug which was not returning the correct state close before a given timestamp if the statelist contained only one state
* added sorted Container for States and Measurements and completely replaced the ringBuffer for the State, compiling, but not tested
* on the way to implement the state_pqueue and meas_pqueue
* implemented SSF vicon filter, which gives same result as ssf_updates version
* moved and renamed measurements and measurement handler classes
* const corrected all interfaces, made most accessors private, except those for aux states
* renamed namespaces and headers to msf
* renamed namespaces and headers to msf
* Contributors: Markus Achtelik, Simon Lynen, asl, georgwi, markusachtelik, omaris, simonlynen
