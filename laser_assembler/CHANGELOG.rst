^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_assembler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.4 (2016-04-11)
------------------
* properly link to gtest
* Contributors: Vincent Rabaud

1.7.3 (2015-04-25)
------------------
* get the test as a proper rostest and clean CMake
  that fixes `#7 <https://github.com/ros-perception/laser_assembler/issues/7>`_
* Merge pull request `#4 <https://github.com/ros-perception/laser_assembler/issues/4>`_ from bulwahn/hydro-devel
  make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Contributors: Lukas Bulwahn, Vincent Rabaud

1.7.2 (2014-04-08)
------------------
* Merge pull request `#3 <https://github.com/ros-perception/laser_assembler/issues/3>`_ from bulwahn/hydro-devel
  check for CATKIN_ENABLE_TESTING
* check for CATKIN_ENABLE_TESTING
* Contributors: David Gossow, Lukas Bulwahn

1.7.1 (2013-06-26)
------------------
* version 1.7.0
* removed rosbuild files
* switched to catkin
* Merge pull request `#2 <https://github.com/ros-perception/laser_assembler/issues/2>`_ from jspricke/point_cloud2_assembler
  New point_cloud2_assembler for PointCloud2 input
* New point_cloud2_assembler for PointCloud2 input
* Contributors: David Gossow, Jochen Sprickerhof, Vincent Rabaud

1.5.5 (2012-11-19)
------------------
* Merge pull request `#1 <https://github.com/ros-perception/laser_assembler/issues/1>`_ from YoheiKakiuchi/groovy-devel
  Adding code for using PointCloud2
* add code for using PointCloud2
* add AssembleScans.srv and add code for using PointCloud2
* Contributors: Vincent Rabaud, applications

1.5.4 (2012-10-10)
------------------
* created stack.xml and added stuff for unary-stack-ification
* add missing boost linkage
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@40155 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* more missing boost links
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38625 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* use the new bullet and eigen conventions
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38342 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing unnecessary exports
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@35366 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing deprecation warnings
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@35255 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Moving away from deprecated API of laser projector
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@35240 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Added Ubuntu platform tags to manifest
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@29657 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* fixed all warnings
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@29627 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding test to laser_assembler. Trac `#3153 <https://github.com/ros-perception/laser_assembler/issues/3153>`_
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26843 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* laser_assembler: adding comments to srv file based on wiki docs
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26802 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Updating stack/manifest.xml files
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26801 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* A little more error checking. `#2126 <https://github.com/ros-perception/laser_assembler/issues/2126>`_
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26581 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Remove use of deprecated rosbuild macros
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@25975 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Added link against boost::system, to fix build on OS X
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@25628 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Updating wiki images
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24787 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Updating example to use assemble_scans
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24653 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Deprecating build_cloud. Renaming to assemble_scans
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24644 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Fixing timing logic in periodic snapshotter
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24625 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding an examples folder in laser_assembler.  Adding a periodic_snapshotter for to be used in tutorials & use case testing
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24624 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Updating laser_assembler manifest to 'api cleared'
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24322 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing grab_cloud_data test app from laser_assembler
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24321 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Clearing out laser_assembler doxygen mainpage
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24320 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Diagram updates
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24319 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Changes as per API Review. Creating nodes without _srv and ROS API Changes
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24296 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Remove inclusion of ros/node.h
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24176 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing calls deprecated code (`#2832 <https://github.com/ros-perception/laser_assembler/issues/2832>`_). Fixing indentation.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24168 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Tweaking laser_assembler manifest
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24075 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding diagrams for laser_assembler
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24071 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Deprecating merge_clouds
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24070 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* laser_assembler [finally] uses NodeHandle instead of Node. Ticket `#1815 <https://github.com/ros-perception/laser_assembler/issues/1815>`_
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@23923 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Merging in remaining missing contents for laser_piple that svn ignored on the first merge.
  git-svn-id: https://code.ros.org/svn/ros-pkg/pkg/trunk/stacks/laser_pipeline@23510 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Contributors: Brian Gerkey, Dave Hershberger, Eitan Marder-Eppstein, Jeremy Leibs, Josh Faust, Ken Conley, Radu Rusu, Rob Wheeler, Vijay Pradeep, Vincent Rabaud
