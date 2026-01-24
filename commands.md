ros2 topic pub --once /speech_request std_msgs/msg/String \
'{"data":"{\"text\":\"Hello. This is Stretch speaking.\",\"volume\":60,\"rate\":170,\"voice\":\"auto\"}"}'