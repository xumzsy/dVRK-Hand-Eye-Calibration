function g = pose2g(pose)
t = [pose.Position.X pose.Position.Y pose.Position.Z]';
q = [pose.Orientation.W pose.Orientation.X pose.Orientation.Y pose.Orientation.Z];
g = [quat2rotm(q) t;0 0 0 1;];