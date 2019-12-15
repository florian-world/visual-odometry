clear all
% close all
clc

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
malaga_path = '../data/malaga-urban-dataset-extract-07/';
kitti_path = '../data/kitti/';

global K PATCHRADIUS MATCHING_THRESHOLD MAGIC_KEYFRAME_THRESHOLD ...
    MAGIC_KEYFRAME_ANGLE_RAD COLOR_LANDMARK COLOR_CANDIDATE ...
    COLOR_TRAJECTORY KEYFRAME_TRANSLATION PIX_TO_RAD FRAME_NUM FIRST_KEYFRAME TOT_TRANSLATION
%PIX_TO_RAD     factor of pixel to degree assuming equal x,y foc lenght
FIRST_KEYFRAME = 3
FRAME_NUM = 1;
TOT_TRANSLATION=[0,0,0]';
KEYFRAME_TRANSLATION = [0,0,0]';
PATCHRADIUS = 10;
MATCHING_THRESHOLD = 0.05;
MAGIC_KEYFRAME_THRESHOLD = 0.15;
MAGIC_KEYFRAME_ANGLE_RAD = deg2rad(5);

COLOR_LANDMARK = 'red';
COLOR_CANDIDATE = 'green';
COLOR_TRAJECTORY = 'black';

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end


PIX_TO_RAD=atan(K(1,3)/(2*K(1,1)))/K(1,3);

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1 3]; % for kitti

if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

initState = bootstrap(img0,img1);

% prepare for continuos operation
state = initState;

trajectory = [0 0 0]; % x y z

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
% First previous image
prevImage = img1;
% visualization in figure 1 (all handled in loop)
figure(1);


for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    
    subplot(1, 3, [1 2]);
    imshow(image);
    hold on;
    [state, pose] = processFrame(state, prevImage, image);
    hold off;
    title(sprintf('Keypoints frame %d', i));
    
    t = -pose(:,4);
    newpoint = trajectory(end,:) + t';
    trajectory(end+1,:) = newpoint;
    
    subplot(1, 3, 3);
    hold on;
    scatter3(state.Landmarks(1, :), state.Landmarks(2, :), state.Landmarks(3, :), 5, COLOR_LANDMARK, 'filled');
    plot3(trajectory(end-1:end,1),trajectory(end-1:end,2),trajectory(end-1:end,3), 'Color', COLOR_TRAJECTORY, 'LineWidth', 2);
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis([-150 150 -10 10 -1 400]);
    hold off;
    
    % Makes sure that plots refresh.    
    pause(0.01);
    prevImage = image;
    FRAME_NUM = FRAME_NUM+1;
end