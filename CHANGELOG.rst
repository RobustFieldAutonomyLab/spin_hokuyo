^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spin_hokuyo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2017-07-21)
------------------
* Removed dyanmixel files in favor of using dependencies as the apt version now apparently works
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Fixed packages in launch files, as well as renamed and organized
* Update README.md
* Fixed dynamixel requirement to be subpackages
* Added dynamixel_controllers to package.xml depends
* Added dynamixel_controller to package.xml depends
* Removed pcl as a dependent because it isn't a package
* Added even more build depends to package.xml
* Added required build dependencies to package.xml
* Update package.xml
* commenting update
* Acknowledgements Update
* Update README.md
* commenting update
* commenting update
* commenting update
* commenting update
* commenting updates
* Delete basic_motors.launch~
* commenting updates
* CMakeLists update
* commenting updates
* commenting update
* Update CMakeLists.txt
* Delete hokuyo_ground_filter.cpp
* Delete floor_max_z.cpp
* commenting update
* commenting update
* commenting update
* comment update
* remove backup file
* remove backup file
* adding back CMakeLists
* Temporarilly removed CMakelist.txt to satisfy failed merge
* organization
* Delete CMakeLists.txt
* moved dynamixel controllers
* moved scan_to_pcl
* Removed rviz config files and fixed tilting lidar continuous launch file
* file location update
* tutorials update
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Renamed misc files and updated launch files and CMakeLists accordingly
* Delete combine_clouds.cpp
* Update CMakeLists.txt
* Update CMakeLists.txt
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Removed extraneous launch files
* name updates
* Rename combine_clouds_subscriber.cpp to point_cloud_assembler.cpp
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Renamed hokuyo_filter to hokuyo_robot_filter to fit the naming convention of hokuyo_ground_filter
* rename for clarity
* typo
* file renames
* Delete combine_clouds.cpp
* Removed floor removal parameter from octomap
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Removed Plane Extract Ransac from tilting lidar and made minor modifications to launch and cpp files
* reset clouds
* Modified combined clouds to allow for parameters and options between time and subscription, changed launch files to allow for further customizations
* Added support for hokuyo_ground_filter
* added tolerance
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* remove ground
* Modified launch files for more universal use and added segmentation launch file and rviz config
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Modified to allow for parameter value to change from dynamixel tilting callback to timer callback
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
  Conflicts:
  tilting_lidar_clean/launch/launch_with_octo/velodyne_hokuyo_octo_tilt_offline.launch
* plane extraction
* Disabled Voxel grid as it is uneccesary for non-assembled point clouds
* Changed name argument to path, requiring full path to be input at launch but allows for universal use across systems
* Removed rosbag node from online launch file
* Added launch files for real time octomaps using voxel grid mapping to reduce lag and missed scans
* Modified offline launch file and added secondary file that accepts point clouds in real time instead of waiting for the full assembled cloud
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* CHANGELOGs added
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Modified offline version of combined octomap generator and created rviz config file
* tutorials update
* Renamed package.xml in tutorials package to prevent ros from reading it due to broken status
* Update README.md
* Update README.md
* Update README.md
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* comment updates
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
  Conflicts:
  tilting_lidar_clean/launch/tilting_lidar_continuous.launch
* Finalized filtered hokuyo scans and added to all launch files
* Fixed tf values for hokuyo to velodyne
* Added static tf between servo and velodyne on all launch files excluding subscriber without octo
* Moved octomap_mapping to launch_with_octo folder
* Used for offline visualization in rviz
* Changed hokuyo_filtered back to scan
* Removed ROS_STREAMs that caused heavy cpu load
* Fixed old octomap launch file
* Fixed duplicate names on relay nodes
* Fixed changes undone by Sarah's second folder confusing git
* removes robot points from scan
* Added modified launch file that creates octomap using velodyne and hokuyo data
* Renamed tilting lidar launch files with octomap to prevent naming conflicts in ros
* Commented octomap out of launch files and created folder explicitly for such files
* Added folder for launch files with octomap
* Changed servo frame to map for better accuracy when moving jackal
* depricated
* name update
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* depricated
* Changed servo fixed frame to map
* Renamed help file back to .cpp and moved to tutorial folder
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Removed note that has been integrated into readme
* Update README.md
* Edited to include link to dynamixel source code
* Marked file for deletion
* Marked file for possible deletion
* Remove outdated README from previous github repo
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Moved outdated launch files to old_launch_files folder
* Removed ROS streams used for debugging
* Removed ROS streams used for debugging
* Changed fixed frame from "camera" to "servo"
* Altered formatting to improve readibility and consistency
* Changed cloud assembler and octomap frames from "camera" to "servo"
* laser assembler update
* hokuyo node update
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Changed servo tf frame from "camera" to "servo"
* update
* direction updates
* Removed excess dynamixel driver files
* Made executable
* misplaced backup file
* Misplaced backup file
* commenting updates
* commenting update
* commenting updates
* Redundant
* Redundant
* commenting update
* Redundant
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* initialize updates
* launch files and intialize update
* Subscriber Updates
* Single Sweep Subscriber Update
* Added dynamixel_motor requirement
* tilt motor subsriber node
* Condensed to single callback function with spin() instead of while loop and spinOnce()
* combined point clouds and octomap update
* single sweep update
* Cloud Compiler Updates
* Subscribing Cloud Compilation Service
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Merge branch 'master' of https://github.com/RobustFieldAutonomyLab/spin_hokuyo
* Single Sweep with Compiled Cloud
* Added comments to wait for inits
* yaml file
* Info Stream Update
* Combining Point Clouds
* Launch file for octomap
* Smooth tilting of lidar
* Add files via upload
* Initial commit
* Contributors: Paul Szenher, Sarah Bertussi
