This ros2 package aims to illustrate problems experienced with rviz2, tf2 and the message MarkerArray. 

The following preconditions need to me met, to experience the problem: 

1. A MarkerArray is published on a frame A
2. Transform is published from frame A to frame B
3. The transform is slightly (e.g. 50ms) newer than the MarkerArray
4. rviz2 uses frame B as Fixed frame

These preconditions leed to the markers not being displayed at all OR being displayed for a brief moment before disappearing again. The error message in rviz2 says that the transform cannot be extrapolated to the future.

Alongside the MarkerArray this package also displays 10 Markers using the Marker message. These are shown without any problems. When publishing the MarkerArray on the Fixed frame, there are no errors and the MarkerArray is displayed.

Use 'ros2 launch rviz_marker launch_rviz_marker_fail.py' and 'ros2 launch rviz_marker launch_rviz_marker_success.py' to run the respective nodes. The launch file launch_rviz_marker_fail.py will publish the Marker and the MarkerArray on a frame differnt to the Fixed frame, while launch_rviz_marker_success.py will display them on the Fixed Frame. Use the rviz2 config in the folder rviz to reproduce the error.

This is the output of the successful and failing launch files. 
<img width="1822" alt="Bildschirmfoto 2024-05-03 um 11 37 41" src="https://github.com/johannesn/rviz_marker/assets/6796392/2a146d38-3810-4533-8175-04d04f4b44b4">
<img width="1778" alt="Bildschirmfoto 2024-05-03 um 11 16 35" src="https://github.com/johannesn/rviz_marker/assets/6796392/5ef3e3f9-8f05-4850-8396-40433425b75f">

This video illustrates the flickering of the MarkerArray:

https://github.com/johannesn/rviz_marker/assets/6796392/b559728a-ff8a-4007-b414-fa4d186a34be

