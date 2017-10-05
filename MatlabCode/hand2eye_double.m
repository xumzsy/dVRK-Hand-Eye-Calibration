% Setting and Parameters
M = 8;     % Number of poses
N = 10;    % Number of data in one pose

% init ROS
disp('begin to initialize ros');
rosinit;
marker_sub_1 = rossubscriber('/aruco_marker_publisher_1/PoseStamped'); % geometry_msgs::PoseStamped
marker_sub_2 = rossubscriber('/aruco_marker_publisher_2/PoseStamped'); % geometry_msgs::PoseStamped

dvrk_pose_sub = rossubscriber('/dvrk/PSM1/position_cartesian_current'); % geometry_msgs::PoseStamped
dvrk_state_sub = rossubscriber('/dvrk/PSM1/robot_state'); % std_msgs::String
disp('Success to subscribe');

% get M poses
marker_pose_1 = zeros(M,7);
marker_pose_2 = zeros(M,7);

robot_pose = zeros(M,7);
pose_count = 0;


while (pose_count < M)
    robot_state = '';
    % block to wait for robot_state to become ready
    while (~strcmp(robot_state,'DVRK_READY'))
        disp('Waiting for Robot Ready')
        robot_state = receive(dvrk_state_sub);
        robot_state = robot_state.Data;
        pause(0.1);
    end
    
    disp('Begin to collect Data');
    marker_buffer_1 = zeros(N,7);
    marker_buffer_2 = zeros(N,7);

    dvrk_buffer = zeros(N,7);
    for i=1:N
        current_marker_pose_1 = receive(marker_sub_1);
        current_marker_pose_2 = receive(marker_sub_2);

        current_dvrk_pose = receive(dvrk_pose_sub);
        marker_buffer_1(i,1) = current_marker_pose_1.Pose.Position.X;
        marker_buffer_1(i,2) = current_marker_pose_1.Pose.Position.Y;
        marker_buffer_1(i,3) = current_marker_pose_1.Pose.Position.Z;
        marker_buffer_1(i,4) = current_marker_pose_1.Pose.Orientation.W;
        marker_buffer_1(i,5) = current_marker_pose_1.Pose.Orientation.X;
        marker_buffer_1(i,6) = current_marker_pose_1.Pose.Orientation.Y;
        marker_buffer_1(i,7) = current_marker_pose_1.Pose.Orientation.Z;
        
        marker_buffer_2(i,1) = current_marker_pose_2.Pose.Position.X;
        marker_buffer_2(i,2) = current_marker_pose_2.Pose.Position.Y;
        marker_buffer_2(i,3) = current_marker_pose_2.Pose.Position.Z;
        marker_buffer_2(i,4) = current_marker_pose_2.Pose.Orientation.W;
        marker_buffer_2(i,5) = current_marker_pose_2.Pose.Orientation.X;
        marker_buffer_2(i,6) = current_marker_pose_2.Pose.Orientation.Y;
        marker_buffer_2(i,7) = current_marker_pose_2.Pose.Orientation.Z;

        dvrk_buffer(i,1) = current_dvrk_pose.Pose.Position.X;
        dvrk_buffer(i,2) = current_dvrk_pose.Pose.Position.Y;
        dvrk_buffer(i,3) = current_dvrk_pose.Pose.Position.Z;
        dvrk_buffer(i,4) = current_dvrk_pose.Pose.Orientation.W;
        dvrk_buffer(i,5) = current_dvrk_pose.Pose.Orientation.X;
        dvrk_buffer(i,6) = current_dvrk_pose.Pose.Orientation.Y;
        dvrk_buffer(i,7) = current_dvrk_pose.Pose.Orientation.Z;
    end
    disp('Get one data');
    sigma_marker_1 = max(std(marker_buffer_1))
    sigma_marker_2 = max(std(marker_buffer_2))

    sigma_dvrk_pose = max(std(dvrk_buffer))
    if max(sigma_marker_1) < 0.001 && max(sigma_marker_2) < 0.001 && max(sigma_dvrk_pose) < 0.001
        % get a valid static pose
        pose_count = pose_count + 1;
        marker_pose_1(pose_count,:) = mean(marker_buffer_1);
        marker_pose_2(pose_count,:) = mean(marker_buffer_2);

        robot_pose(pose_count,:) = mean(dvrk_buffer);
        disp('Successfully Get a New Pose!');
    else
        disp('Current pose is not stable');
    end
    
end
disp('Successfully get all the data and begin to shutdown ros');
rosshutdown;

A = zeros(4,4*M);
B1 = zeros(4,4*M);
B2 = zeros(4,4*M);

for i=1:M
    A(:,4*i-3:4*i) = [quat2rotm(robot_pose(i,4:7)) robot_pose(i,1:3)';0 0 0 1;];
    B1(:,4*i-3:4*i) = [quat2rotm(marker_pose_1(i,4:7)) marker_pose_1(i,1:3)';0 0 0 1;];
    B2(:,4*i-3:4*i) = [quat2rotm(marker_pose_2(i,4:7)) marker_pose_2(i,1:3)';0 0 0 1;];

end
[X1,Y1] = shah(A,B1);
[X2,Y2] = shah(A,B2);
