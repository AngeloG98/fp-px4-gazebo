clear
rosrun offboard_node offboard_node & PID1=$!
sleep 5s
roslaunch plan_manage topo_replan.launch & PID2=$!

# exit
wait
kill PID1 PID2
exit