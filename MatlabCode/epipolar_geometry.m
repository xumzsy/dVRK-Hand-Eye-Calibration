function gr =  epipolar_geometry(gc1,gc2,Y1,Y2,P1,P2)
% Given a point in 2 image from two camera, solve its position in world
% gc1 & gc2: [u,v], position in two images, 2*1 vector
% Y: camera position 4*4, relative to robot base
% P: camera information 12*1

% Reshape P to 3*4 projection matrix
P1 = [P1(1:4)';P1(5:8)';P1(9:12)'];
P2 = [P2(1:4)';P2(5:8)';P2(9:12)'];
Y1(1:3,4) = Y1(1:3,4)*1000;
Y2(1:3,4) = Y2(1:3,4)*1000;

% [u,v,w]' = U*[xr yr zr 1]'
U1 = P1/(Y1);
U2 = P2/(Y2);

% A*[xr yr zr]'=b
A = zeros(4,3);
A(1,:) = [U1(1,1)-gc1(1)*U1(3,1), U1(1,2)-gc1(1)*U1(3,2), U1(1,3)-gc1(1)*U1(3,3)];
A(2,:) = [U1(2,1)-gc1(2)*U1(3,1), U1(2,2)-gc1(2)*U1(3,2), U1(2,3)-gc1(2)*U1(3,3)];
A(3,:) = [U2(1,1)-gc2(1)*U2(3,1), U2(1,2)-gc2(1)*U2(3,2), U2(1,3)-gc2(1)*U2(3,3)];
A(4,:) = [U2(2,1)-gc2(2)*U2(3,1), U2(2,2)-gc2(2)*U2(3,2), U2(2,3)-gc2(2)*U2(3,3)];

b = [gc1(1)*U1(3,4)-U1(1,4),gc1(2)*U1(3,4)-U1(2,4),gc2(1)*U2(3,4)-U2(1,4),gc2(2)*U2(3,4)-U2(2,4)]';
gr = linsolve(A,b)/1000;
end