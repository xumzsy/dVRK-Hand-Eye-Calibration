%% Initialize ROS
rosinit;

%% Get Camera Infomation
info_sub1 = rossubscriber('camera1/color/calibrated_camera_info');
info_sub2 = rossubscriber('camera2/color/calibrated_camera_info');
info1 = receive(info_sub1);
info2 = receive(info_sub2);
P1 = info1.P;
P2 = info2.P;
%% Hand Eye calibration
hand2eye_double;

%%
stereo_calibration_marker_script;
%% Get Image rgb1 and rgb2;
rgb_sub1 = rossubscriber('camera1/color/image_rect_color');
rgb_sub2 = rossubscriber('camera2/color/image_rect_color');

rgb_1 = receive(rgb_sub1);
rgb_2 = receive(rgb_sub2);

rgb1 = readImage(rgb_1);
rgb2 = readImage(rgb_2);

%%  Maually Click to pick Corners
N=4;
imshow(((rgb1)));
gc1 = ginput(N);

imshow((rgb2));
gc2 = ginput(N);

%% Automatically detect corners (may need manually pair them)
[result_A, result_B,mask_A,mask_B] = detectCorners(rgb1, rgb2, 0 ,0);


%% Solve corners in robot space
gr = zeros(3,N);

N=4;
for i=1:N
    % gr(:,i) =   epipolar_geometry(gc1(i,:),gc2(i,:),Y1,Y2,P1,P2);  %
    % Manual Pick up Corners 
    gr(:,i) =  epipolar_geometry(result_A(i,:),result_B(i,:),YG1,YG2,P1,P2);

end
