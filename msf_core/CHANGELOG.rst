^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package msf_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fixed glog find script to find glog on osx
* removed -Werror flag since recent boost versions spit out a warning about deprecated boost::signals
* Fixed glog exporetd linker flags. Ubuntu 12.04 users will have to install glog from source.
* Add replacement for Glog
* Split similarity transform tests into separate test cases
* Adding unit test similarity transform
* stricter compiler warning settings and Werror
* Changes for compatibility with gcc4.6 on the build server
* fixed matrix block reference issue and corrected formatting
* added publishers for covariances
  -added DaoubleMatrixStamped message
  -publishers for core state covariance, aux state covariance, and core-aux cross-cov
  -extended state definition and counters for core error state counting
* cast nError States for dynamic size matrices
* changed dyn size measurement base apply correction
* added options to statevar
* added msg dropout watchdogs and increase queuesizes. can process 750Hz
  IMU, 450Hz vision and 150Hz GPS without dropouts.
* added gps and changed pointwCov
* correct state timestamps for published states
* the position-pose sensor still needs debugging for the initial values.
* compiles -Wall -Wextra -pedantic without warnings
* changed imu->cam calibration indices to fit the new imu-centered
  notation. changed world->vision drift to fit the new world-centered
  notation
* changed position world vision offset to correct notation and adjusted
  indices
* removed all -O2 from child directories, so we can control the build type
  from the top dir. Ros Release build types are anyway O2
* fix bug nonclean reset, caused by dirty state of fuzzytracking checker
* fixed q_vw pose sensor bug
* moved tests
* wait for thread sync on init
* reorganized all parameters
* another switch for cov image publishing
* compile time disable cov image pub
* vision-gps sensor draft
* added warning on no meas applied
* Merge branch 'master_eval' into sc
  Conflicts:
  msf_updates/src/pose_msf/pose_sensormanager.h
* the falsecolor code
* finally got the Matlab jet colormap working, what a pain
* renamed msf_tools
* file renames, moved falsecolor to lib
* added a matlab - jet like colormap and using that in msf_core now
* added clear cross cov on state fix, will do pseudo meas later as a
  better solution
* added cov image publisher
* SC seems to be working, needs comparison with absolute meas and some
  optimization in apply-corr-relative
* evalparams
* params
* added asctec imu callback and moved imu processing to new function, so
  that it can be called from multiple cbs
* Merge branch 'master' of git@github.com:ethz-asl/sensor_fusion.git
* dropping measurements that are earlier than the first state in the
  buffer. (vicon meas with delay)
* Merge branch 'master' of git@github.com:ethz-asl/sensor_fusion.git
* added yet another check for data playback being on and the EXTEKF
  having subscribers
* ext EKF fix bug no publish on state nan check
* changed fuzzytracking interface
* added checks on state before publishing to EXT EKF
* Merge branch 'master' of git@github.com:ethz-asl/sensor_fusion.git
  Conflicts:
  msf_core/include/msf_core/msf_tools.h
* added preparations for SC. Mainly sensor IDs for measurements
* moved humantime to cpp file and made a library; cmakelists and manifext adjusted accordingly
* changed def for median and timehuman
* and back .. removed inclusion
* added tools inclusion to fwds
* changed output indexing for state print
* another change to state print formatting
* minor change to state print formatting
* moved state echo after init to core
* added print function for state
* move P core init to impl
* added reset call in ctor
* default value for state visitor on state reset
* added std=c++0x to exports in manifest
* fixed include bug in msf_fwds.h
* made propagateState public
* added prediction check for same states and removed two warnings
* added specialization for getenumtype with index -1, which is needed for
  nontempdrifting state index if non is actually defined
* made specialization checkfuzzy public
* moved fuzzy tracking to separate class, so that msf can be used without
  nontemporal drifting state.
* removed -W-no-string-compare
* added Quaternion type and removed commented stuff
* made predictProcessCovariance public and removed unnecessary
  Fd.setIdentity()
* minor reorganized header inclusions, removed ros calls in subdir cmake
  files
* Fd Qd from member to local variable
* init pose sensor handler to zero/unit, minor changes to remove warnings
* Added warning if message throttling is on
* fixed the error that the hl-state messages where overwriting the
  covariance of the init state if the after the init there was another
  message arriving from the hl with the old state values.
* fixed timing issues when used with ext propagation. Now working with
  pressure sensor and height init.
* changed correction index calculation to a new simpler to use meta
  programming struct
* fixed some documentation
* moved pvw next to qvw in the state definition and adapted the plotting scripts
* added plot for both attitude and position vision-world drift
* added vision-world position drift state
* added a check for positive definite covariance matrix in pose sensor
* adding aslam config files, set frameid of published pose to /world
* minor change to saveguard the init procedure from other
  threads applying measurements
* added additional checks for EKF reset in the add-measurements method
* minor
* small typename fix
* popping queue instead of clear
* resetting state buffer upon first external state insertion and echo of
  the state buffer contents plus the return value from the hl
* added state echo and EKF panic on NaN in propagation
* added some more documentation
* removed Const typedefs and fixed affected files
* adapted similaritytransform to new typedefs
* added covariance block methods and tests
* added method to compute Xi matrix for quaternion multiplication
* added files for pose/scale initialization
* added additional safety checks for numeric problems, still with some init requests the initial state is containing some NaNs
* Merge branch 'modular_sensor_fusion' of github.com:ethz-asl/sensor_fusion into HEAD
* added measurement queue for cases where the propagation is not ready when a measurement arrives
* removed bug in sortedContainer to check for value at time if closest to time is requested. changed pressure sensor to queue measurements
* removed bug in pose pressure sensor, wrong sign of measurement matrix entry for pressure sensor. Still the delay compensation has some problems, spiking
* changed H matrix calculation for pressure sensor
* changed header file extensions to match ROS code style
* fixed a bug in the multi-sensor filter with state interpolation and
  creation of new states at the time of the measurement. this might also
  have fixed the multi-sensor filter fuzzy tracking, but not tested
* implemented linear state interpolation in cases where the closest state
  is too far away from the measurement
* pressure sensor not working, fixed propagation infinite loop in applymeas of core
* changed enum values of the state and renamed state member variables.
  Finished impl of pressure-pose sensor and refactored the header
  organization of msf_statedef
* added static assertion checker for core state ordering in separate file, core is now completely free of state definitions
* moved state definition from core to msf_updates subdirs to be defined for every single sensor
* templated measurements with EKFState_T
* templated core classes to EKFState type
* changed namespace of statedef to msf_updates
* changed formatting to ros style file
* catching bad config with data playback now
* Merge branch 'modular_sensor_fusion' of github.com:ethz-asl/sensor_fusion into modular_sensor_fusion
* covariance propagation in state callback
* removed delay comment for dox
* dynamic reconfigure reoorganized, delay moved to measurement, works with tutorial dataset
* changed meas covariance bug, which was not setting the member variable, minor cleanup
* initial version of a pose sensor handler, with overloads for tf, pose, posewcov
* modified ext ekf size and working on pose sensor, but not finished
* remove libexport from msf_core
* added doxygen config and layout file and continued on the documentation
* minor chnages to the documentation and added EIGEN_MAKE_ALIGNED_OPERATOR_NEW to some classes
* modified the documentation to include more detailed tags for doxygen
* added documentation to all methods/classes/variables
* added option to use simulated core covariance and fixed diag. for aux states. Alternatively the user can still provide a full simulated cov. matrix as before
* changed stack descr. and removed some small parts in the code which were unused
* removed old code parts, removed debugging output and cleaned up. No functionality changes
* added a janitor which cleans the buffers if they contain states/meas older than some defined value
* msf working with vicon dataset. Giving the exact same results as ssf with the new infinite statebuffer
* still crashing with some init values. It seems that the selection of states for the update is not done correctly sometimes
* added liscense to all files. Debugging stateMap and measMap. Prediction ok, update not working so far
* fixed initialization to use a measurement object and removed the bug which was not returning the correct state close before a given timestamp if the statelist contained only one state
* added sorted Container for States and Measurements and completely replaced the ringBuffer for the State, compiling, but not tested
* on the way to implement the state_pqueue and meas_pqueue
* implemented SSF vicon filter, which gives same result as ssf_updates version
* moved and renamed measurements and measurement handler classes
* moved fuzzy tracking detection back to core and added compile time lookup for the best state to use as non temporal drifting state.
* const corrected all interfaces, made most accessors private, except those for aux states
* compiling and all ToDos done, initialization of the filter and the update part (from the sensor side) is still undone
* added some files that resulted from the reorganisation of some structure to allow fwds to work better
* changed the StateVar Type again to reflect the idea of having multiple types of statevars that are core/core with propagation or auxiliary.
* moved usercalculations to base class and state implementation to separate header, still not everything is implemented
* changed all state access functions, compiling, still got to implement correction and some other ToDos
* parametrized stateVar_T for propagation change and added more unit testing, still there are a lot of compiler errors
* still many injuries in the template war
* in the middle of a template war
* in the middle of a template war
* bugfix for startindex calculation caused by const ref types from boost::fusion::c_at type resolve. Added typetraits for qualifier stripping, adding
* finished changes to msf_state, now reflecting the full functionality of former 'state' class
* backup, state implementation not finished
* backup, state implementation not finished
* added gtest for all current compile time computations
* added another main header for msf_core
* replaced all Eigen::Vector3d with generic Eigen::Matrix type and removed double state_T
* bugfix calculation of start indices for a given state
* still a bug in the offset calculation, there is something added twice
* moved tmp and all other definitions to separate header files
* added compile time offset calculation for full state and correction vector. added static assertion checking correct ordering of the state and the enum
* added some first draft of the tmp test executable, still the offsets of the states in the correction and state vector are not clear
* renamed namespaces and headers to msf
* added gtest for all current compile time computations
* added another main header for msf_core
* replaced all Eigen::Vector3d with generic Eigen::Matrix type and removed double state_T
* bugfix calculation of start indices for a given state
* still a bug in the offset calculation, there is something added twice
* moved tmp and all other definitions to separate header files
* added compile time offset calculation for full state and correction vector. added static assertion checking correct ordering of the state and the enum
* added some first draft of the tmp test executable, still the offsets of the states in the correction and state vector are not clear
* renamed namespaces and headers to msf
* Contributors: Markus Achtelik, Simon Lynen, asl, georgwi, markusachtelik, omaris, simonlynen, wueestm
