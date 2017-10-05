function g = get_marker_pose(sub,N)
g = zeros(4);
for i=1:N
    P = receive(sub);
    g = g + pose2g(P.Pose);
    pause(0.1);
end
g = g/N;