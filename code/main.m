%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
malaga_path = '../data/malaga-urban-dataset-extract-07/';
kitti_path = '../data/kitti/';

global K PATCHRADIUS MATCHING_LAMBDA MAGIC_KEYFRAME_THRESHOLD ...
    MAGIC_KEYFRAME_ANGLE_RAD
PATCHRADIUS = 10;
MATCHING_LAMBDA = 4;
MAGIC_KEYFRAME_THRESHOLD = 0.1;
MAGIC_KEYFRAME_ANGLE_RAD = deg2rad(5);

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

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
% visualization stuff
figure(1);
subplot(1, 3, 3);
% scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-15 10 -10 5 -1 40]);


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
    
    
    [state, pose] = processFrame(state, image);
    
    subplot(1, 3, [1 2]);
    imshow(image);
    hold on;
%     plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
%         matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
%     if (nnz(inlier_mask) > 0)
%         plot(matched_query_keypoints(2, (inlier_mask)>0), ...
%             matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
%     end
%     plotMatches(corresponding_matches(inlier_mask>0), ...
%         matched_query_keypoints(:, inlier_mask>0), ...
%         keypoints);
    hold off;
    title('Inlier and outlier matches');
    
    
    % Makes sure that plots refresh.    
    pause(0.01);
    prevImage = image;
end