close all
clear
clc

% load ROS bag
bag = rosbag("data_2023-11-24-13-35-49.bag");

% error plotting
err = bag.select("Topic","/iiwa/error");
errTs = timeseries(err);
plot(errTs)

% torques
for i = 1:7
    topicName = "/iiwa/iiwa_joint_" + string(i) + "_effort_controller/command";
    torqueTs = timeseries(bag.select("Topic", topicName));
    figure
    plot(torqueTs), hold on
end
hold off