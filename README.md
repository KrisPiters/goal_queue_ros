# goal_queue_ros

A simple ROS node which stores poses (received on topic `/goal_queue_goals`) in a queue.
The poses in the queue are send, as navigation goals for `move_base`, in an endless loop using a simple action client.

.
