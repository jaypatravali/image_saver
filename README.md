Commands

Run: rosrun image_saver data_writer  _image_transport:=compressed _approximate_sync:=True
Compile: catkin_make -DCMAKE_BUILD_TYPE=Release
Bags: rosbag play  -l -s 40 -r 2 ais_loop.bag
Clear:   find . ! -name '.*' ! -type d -exec rm -- {} +


TODO

1. Cleanup
2. 16 messages.


