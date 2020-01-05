clear all
close all
clc

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
malaga_path = '../data/malaga-urban-dataset-extract-07/';
kitti_path = '../data/kitti/';

global K PATCHRADIUS MATCHING_THRESHOLD MAGIC_KEYFRAME_THRESHOLD ...
    MAGIC_KEYFRAME_ANGLE_RAD COLOR_LANDMARK COLOR_CANDIDATE ...
    COLOR_TRAJECTORY PIX_TO_RAD KEYFRAME_THRESHOLD

% main config (PIX_TO_RAD and K set below, depending on dataset)
KEYFRAME_THRESHOLD = 60;
PATCHRADIUS = 10;
MATCHING_THRESHOLD = 0.05;
MAGIC_KEYFRAME_THRESHOLD = 0.14;
MAGIC_KEYFRAME_ANGLE_RAD = deg2rad(4);
COLOR_LANDMARK = 'red';
COLOR_CANDIDATE = 'green';
COLOR_TRAJECTORY = 'black';

location=[0,0,0];
orientation=[{eye(3)}];
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
bootstrap_frames = 1:3; % for kitti

bootstrap_imgs = cell(length(bootstrap_frames));

if ds == 0
    for i = bootstrap_frames
        bootstrap_imgs{i} = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(i))]);
    end
elseif ds == 1
    for i = bootstrap_frames
        bootstrap_imgs{i} = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(i)).name]));
    end
elseif ds == 2
    for i = bootstrap_frames
        bootstrap_imgs{i} = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(i))]));
    end
else
    assert(false);
end

initState = bootstrap(bootstrap_imgs);

% prepare for continuos operation
state = initState;

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
trajectory = zeros(last_frame, 3); % N * [x y z]
% First previous image
prevImage = bootstrap_imgs{length(bootstrap_imgs)};
% visualization in figure 1 (all handled in loop)
figure(1);
set(gcf, 'Position',  [360, 500, 1200, 600]);


HIST_SIZE = 20;
numLandmarksHistory = zeros(1,HIST_SIZE);


for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    fprintf('Ground truth: (%2.1f, %2.1f, %2.1f)\n', ground_truth(i,1),0,ground_truth(i,2));
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
    
    subplot(2, 3, [1 2]);
    tmp = gcf;
    l = tmp.CurrentAxes.get('xlabel').String;
    imshow(image);
    xlabel(l);
    hold on;
    [state, pose] = processFrame(state, prevImage, image);
    hold off;
    title(sprintf('Keypoints frame %d', i));
    
    trajectory(i,:) = pose(:,4)';
    
%     [curOrientation,curLocation] = extrinsicsToCameraPose(pose(:,1:3),pose(:,4));
    
%     curLocation
    
%     location(end+1,:)=location(end,:)+(C_tot*curLocation')';
%     C_tot = C_tot * curOrientation;
%     orientation(end+1)={orientation{end}*curOrientation};
    
    subplot(2, 3, [3 6]);
    scatter3(state.Landmarks(1,:), state.Landmarks(2,:), state.Landmarks(3,:), 3, COLOR_LANDMARK, 'filled');
    hold on;
    plot3(trajectory(1:i,1),trajectory(1:i,2),trajectory(1:i,3), 'Color', COLOR_TRAJECTORY, 'LineWidth', 2); 
%     plot3(ground_truth(1:i,1),zeros(i,1),ground_truth(1:i,2), 'Color', COLOR_TRAJECTORY, 'LineWidth', 1, 'LineStyle', '--');
    xlabel(sprintf("Estimated position: (%2.1f, %2.1f)", pose(1,4), pose(3,4)));
    
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
%     axis vis3d;
    axis([[-10 10] + pose(1,4), -10 10, ([-10 40] + pose(3,4))]);
    hold off;
    
    subplot(2,3,5);
    plot3(trajectory(1:i,1),trajectory(1:i,2),trajectory(1:i,3), 'Color', COLOR_TRAJECTORY, 'LineWidth', 2);
%     plot3(ground_truth(1:i,1),zeros(i,1),ground_truth(1:i,2), 'Color', COLOR_TRAJECTORY, 'LineWidth', 1, 'LineStyle', '--');
    title("Full trajectory");
    view(0,0);
    axis equal;
    grid on;
%     axis([-300 400 -10 10 -10 700]);
%     axis([-20 80 -10 10 -10 100]);
    if (diff(xlim) > diff(zlim))
        m = mean(zlim);
        d = diff(xlim);
        axis([xlim -d/2 d/2 (m - d/2) (m + d/2)]);
    else
        m = mean(xlim);
        d = diff(zlim);
        axis([(m - d/2) (m + d/2) -d/2 d/2 zlim]);
    end
    subplot(2,3,4);
    set(gcf, 'GraphicsSmoothing', 'on');
    frameNumHistory = (-HIST_SIZE+1):0;
    numLandmarks = size(state.Landmarks, 2);
    numLandmarksHistory = circshift(numLandmarksHistory,-1);
    numLandmarksHistory(end) = numLandmarks;
    
    plot(frameNumHistory, numLandmarksHistory,'Color', "black", 'LineWidth', 1);
    title("Number of landmarks in past 20 frames");
    xlabel("frame number");
    ylabel("number of landmarks");
    ylim([0 round(max(numLandmarksHistory)+10,-1)]);
    
    % Makes sure that plots refresh.
    pause(0.05);
    prevImage = image;
end