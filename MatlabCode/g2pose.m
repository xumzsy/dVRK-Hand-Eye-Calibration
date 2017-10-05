function pose = g2pose(g)
% transform 4*4 transformation matrix to geometr_msgs/pose
pose.MessageType='geometry_msgs/Pose';
pose.Position.X = g(1,4);
pose.Position.Y = g(2,4);
pose.Position.Z = g(3,4);
q = rotm2quat(g(1:3,1:3));
pose.Orientation.W = q(1);
pose.Orientation.X = q(2);
pose.Orientation.Y = q(3);
pose.Orientation.Z = q(4);

