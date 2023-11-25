close all
clear
clc

plotErrorAndTorques("data_2023-11-24-13-35-49.bag");



function plotErrorAndTorques(bagname)
    % load ROS bag
    bag = rosbag(bagname);
    
    % error plotting
    err = bag.select("Topic","/iiwa/error");
    errTs = timeseries(err);
    figure("Name","Error")
    plot(errTs)
    grid on
    ylabel("")
    title("Error norm")
    
    % torques
    fig_torques = figure("Name","Torques");
    leg = cell(7,1);
    for i = 1:7
        topicName = "/iiwa/iiwa_joint_" + string(i) + "_effort_controller/command";
        torqueTs = timeseries(bag.select("Topic", topicName));
        figure(fig_torques);
        plot(torqueTs)
        hold on
        leg{i} = "Joint " + i + " torque";
    end
    hold off
    title("Torques")
    legend(leg,"location","sw")
    ylabel("")
    grid on
end