%% Phantom Corner Detector
%   Output:
%   - result_A = coordinates of the four corners for img_A
%   - result_B = coordinates of the four corners for img_B
%       *** Corner coordinates starting from top left corner and cycle
%       clockwise order
%   Input:
%   - img_A = RGB image of left camera
%   - img_B = RGB image of right camera
%   - yes_fig = 0 means no figures/graphs, 1 means yes figures/graphs 
%   - yes_save = 0 means not saving figure images;
%	         numbers bigger than 0 means saving images (with that number in file name)
% *** When the lighting is dimmer and reflections are minimized,
%       this function works better

function [result_A, result_B, mask_A, mask_B] = detectCorners(img_A, img_B,...
    yes_fig, yes_save)

%% Deriving Focal length
fx = 463.889; fy = 463.889;
px = 320; py = 240;
K = [fx 0 0; 0 fy 0; px py 1];

[height, width, ~] = size(img_A);

%% deriving--> mask_A

%hsv_A = rgb2hsv(img_A);
% loop for thresholding the HSV value

img_new_A = img_A;

R_A = reshape(img_A(:,:,1),[],1);

R_A(R_A < 5 | R_A > 175) = [];

thresh_R_A = mode(R_A);

for i = 1:height
    for j = 1:width
        %h = hsv_A(i, j, 1); % hue
        %s = hsv_A(i, j, 2); % saturation
        %v = hsv_A(i, j, 3); % value
        %v <= 0.13 %&& h >= 0.45 && h <= 0.8 % above cyan color (0.5) -> so if color is ~blue
        if img_A(i, j, 1) < thresh_R_A + 15
            img_new_A(i, j, 1) = 0;
            img_new_A(i, j, 2) = 0;
            img_new_A(i, j, 3) = 0; % value = 0 -> pixel is black
        end
    end
end
% Back to rgb image
% img_new_A = hsv2rgb(hsv_A);

if yes_fig == 1
    figure, imshow(img_new_A), title('processed rgb A');
end

gr_A = rgb2gray(img_new_A);

n_A = 1;
Idouble_A = im2double(gr_A);
avg_A = mean2(Idouble_A);
sigma_A = std2(Idouble_A);

input1_A = avg_A-n_A*sigma_A;
input2_A = avg_A+n_A*sigma_A;
if input1_A < 0
    input1_A = 0;
end
if input2_A > 1
    input2_A = 1;
end

adj_A = imadjust(gr_A,[input1_A, input2_A],[]);

gaus_A = imgaussfilt(adj_A,1.15);

%adj_A = imadjust(gr_A,[0.11 0.8],[]);

%{
gaus_A = imgaussfilt(adj_A,1.15);
figure; imshow(gaus_A);

sharp_A = imsharpen(gaus_A,'Radius',2,'Amount',1);
figure; imshow(sharp_A);
%}

bw_A = imbinarize(gaus_A,0.175);

%bw_A = imbinarize(adj_A,0.175);
r_A = height/2;  %%% USER INPUT
c_A = width/2;         %%% USER INPUT
while bw_A(r_A,c_A) == 1
    r_A = r_A + 1;
end
contour_A = bwtraceboundary(bw_A,[r_A-1 c_A],'E',8,Inf);
mask_A = poly2mask(contour_A(:,2), contour_A(:,1), height, width);

if yes_fig == 1
    figure, imshow(mask_A), title('mask A');
end

%% deriving--> mask_B

img_new_B = img_B;

R_B = reshape(img_B(:,:,1),[],1);

R_B(R_B < 5 | R_B > 175) = [];

thresh_R_B = mode(R_B);

for i = 1:height
    for j = 1:width
        %h = hsv_B(i, j, 1); % hue
        %s = hsv_B(i, j, 2); % saturation
        %v = hsv_B(i, j, 3); % value
        %v <= 0.13 %&& h >= 0.45 && h <= 0.8 % above cyan color (0.5) -> so if color is ~blue
        if img_B(i, j, 1) < thresh_R_B + 15
            img_new_B(i, j, 1) = 0;
            img_new_B(i, j, 2) = 0;
            img_new_B(i, j, 3) = 0; % value = 0 -> pixel is black
        end
    end
end
% Back to rgb image
%img_new_B = hsv2rgb(hsv_B);

%sharp_B = imsharpen(img_new_B,'Radius',2,'Amount',1);
if yes_fig == 1
    figure, imshow(img_new_B), title('processed rgb B');
end

gr_B = rgb2gray(img_new_B);

n_B = 1;
Idouble_B = im2double(gr_B);
avg_B = mean2(Idouble_B);
sigma_B = std2(Idouble_B);

input1_B = avg_B-n_B*sigma_B;
input2_B = avg_B+n_B*sigma_B;
if input1_B < 0
    input1_B = 0;
end
if input2_B > 1
    input2_B = 1;
end

adj_B = imadjust(gr_B,[input1_B, input2_B],[]);

gaus_B = imgaussfilt(adj_B,1.15);

%adj_B = imadjust(gr_B,[0.15 0.8],[],1.2);

%{
gaus_B = imgaussfilt(adj_B,1.15);
figure; imshow(gaus_B);

sharp_B = imsharpen(gaus_B,'Radius',2,'Amount',1);
figure; imshow(sharp_B);
%}

bw_B = imbinarize(gaus_B,0.175);

%bw_B = imbinarize(adj_B,0.175);

r_B = 276;%height/2 + 100;  %%% USER INPUT
c_B = 243;%width/2;         %%% USER INPUT
while bw_B(r_B,c_B) == 1
    r_B = r_B + 1;
end
contour_B = bwtraceboundary(bw_B,[r_B-1 c_B],'W',8,Inf);
mask_B = poly2mask(contour_B(:,2), contour_B(:,1), height, width);

if yes_fig == 1
    figure, imshow(mask_B), title('mask B');
end

%gray_A = adj_A.* double(mask_A);
%gray_B = adj_B.* double(mask_B);

edge_A = edge(mask_A);
edge_B = edge(mask_B);

if yes_fig == 1
    figure, imshow(edge_A), title('edge A');
    figure, imshow(edge_B), title('edge B');
end

result_A = corner_finder(mask_A, yes_fig, yes_save, 1);
result_B = corner_finder(mask_B, yes_fig, yes_save, 2);

if yes_fig == 1
    figure, imshow(img_A), hold on
    plot(result_A(:,2), result_A(:,1), 's', 'MarkerSize',10, 'MarkerFaceColor','r')
    hold off, title('Corners')
    if yes_save > 0
        saveas(gcf, strcat('Output_Images/corner_A', int2str(yes_save), '.png'));
    end      
    figure, imshow(img_B), hold on
    plot(result_B(:,2), result_B(:,1), 's', 'MarkerSize',10, 'MarkerFaceColor','r')
    hold off, title('Corners')
    if yes_save > 0
        saveas(gcf, strcat('Output_Images/corner_B', int2str(yes_save), '.png'));
    end  
end

if yes_save > 0
    imwrite(img_A,strcat('Output_Images/img_A', int2str(yes_save), '.png'));
    imwrite(img_B,strcat('Output_Images/img_B', int2str(yes_save), '.png'));
    
    imwrite(img_new_A,strcat('Output_Images/img_new_A', int2str(yes_save), '.png'));
    imwrite(img_new_B,strcat('Output_Images/img_new_B', int2str(yes_save), '.png'));
    
    imwrite(mask_A,strcat('Output_Images/mask_A', int2str(yes_save), '.png'));
    imwrite(mask_B,strcat('Output_Images/mask_B', int2str(yes_save), '.png'));
    
    imwrite(edge_A,strcat('Output_Images/edge_A', int2str(yes_save), '.png'));
    imwrite(edge_B,strcat('Output_Images/edge_B', int2str(yes_save), '.png'));
end
    
end






%% *** Corner Finder Helper Function ***
function result = corner_finder(I, yes_fig, yes_save, AB)

if AB == 1
    cur = 'A';
elseif AB == 2
    cur = 'B';
end

%# get boundary
B = bwboundaries(I, 8, 'noholes');
B = B{1};

%%# boudary signature
%# convert boundary from cartesian to ploar coordinates
objB = bsxfun(@minus, B, mean(B));
[theta, rho] = cart2pol(objB(:,2), objB(:,1));

%# find corners
%corners1 = find( diff(diff(rho)>0) < 0 );     %# find peaks

%thresh = find(rho <= 145 & rho >= 144);

%[~,order] = sort(corners1, 'descend');
%corners = order(1:10);
%[~,order] = sort(rho, 'descend');
%corners = order(1:20);

% Find local minimums
[mins,~,min_w,min_p] = findpeaks(-rho, 'MinPeakProminence', 2);

mins2 = zeros(length(mins),1);
for i = 1:length(mins)
    temp_id = find(-rho == mins(i));
    mins2(i) = temp_id;
end

new_theta_index = find(theta <= theta(mins2(1)));
theta = vertcat(theta, (theta(new_theta_index)+(2*pi)));
rho = vertcat(rho, rho(new_theta_index));

% Find local maximums
[pks,~,max_w,max_p] = findpeaks(rho, 'MinPeakHeight', 115, 'MinPeakProminence', 2);
[~, sorted_id] = sort(pks, 'descend');

corners = zeros(4,1);
result = zeros(4,2);

for i = 1:4
    temp_id = find(rho == pks(sorted_id(i)));
    corners(i) = temp_id(1);
    result(i,1) = B(corners(i),1);
    result(i,2) = B(corners(i),2);
end

if yes_fig == 1
    %# plot boundary signature + corners
    figure, plot(theta, rho, '.'), hold on
    plot(theta(corners), rho(corners), 'ro'),hold off
    xlim([-pi 2*pi]), title('Boundary Signature'), xlabel('\theta'), ylabel('\rho')

    if yes_save > 0
        saveas(gcf, strcat('Output_Images/tr_max_', cur, int2str(yes_save), '.png'));
    end    
    
    figure, plot(theta, rho, '.'), hold on
    plot(theta(mins2), rho(mins2), 'ro'),hold off
    xlim([-pi 2*pi]), title('Boundary Signature'), xlabel('\theta'), ylabel('\rho')
end

% plot minimums

%display(mins2);
%display(mins);

%{
%# plot image + corners
figure, imshow(I), hold on
plot(B(corners,2), B(corners,1), 's', 'MarkerSize',10, 'MarkerFaceColor','r')
hold off, title('Corners')
%}
end
