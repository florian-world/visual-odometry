clear all;
close all;
%rng(1);
keyF=0;

% Create data for parts 1 and 2
num_inliers = 20;
num_outliers = 10;
noise_ratio = 0.1;
poly = rand(3, 1); % random second-order polynomial
extremum = -poly(2)/(2*poly(1));
xstart = extremum - 0.5;
lowest = polyval(poly, extremum);
highest = polyval(poly, xstart);
xspan = 1;
yspan = highest - lowest;
max_noise = noise_ratio * yspan;
x = rand(1, num_inliers) + xstart;
y = polyval(poly, x);
y = y + (rand(size(y))-.5) * 2 * max_noise;
data = [x (rand(1, num_outliers) + xstart)
    y (rand(1, num_outliers) * yspan + lowest)];

% Data for parts 3 and 4
K = load('../data/K.txt');
keypoints = load('../data/keypoints.txt')';
p_W_landmarks = load('../data/p_W_landmarks.txt')';

% Data for part 4
database_image = imread('../data/000000.png');

% Dependencies
addpath('plot');
% Replace the following with the path to your DLT code:
addpath('../../../2/code');
% Replace the following with the path to your keypoint matcher code:
addpath('../../../3/code');

%% Part 1 - RANSAC with parabola model
[best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise);

% Compare with full data fit.
full_fit = polyfit(data(1, :), data(2, :), 2);

figure(2);
subplot(1, 2, 1);
scatter(data(1,:), data(2, :), 'b');
hold on;
x = xstart:0.01:(xstart+1);
for i = 1:length(best_guess_history)-1
    guess_plot = plot(x, polyval(best_guess_history(:, i), x), 'b');
end
truth_plot = plot(x, polyval(poly, x), 'g', 'LineWidth', 2);
best_plot = ...
    plot(x, polyval(best_guess_history(:, end), x), 'r', 'LineWidth', 2);
fit_plot = ...
    plot(x, polyval(full_fit, x), 'r--', 'LineWidth', 2);
axis([xstart xstart+1 lowest-max_noise highest+max_noise]);
legend([truth_plot, best_plot, fit_plot, guess_plot], 'ground truth', ...
    'RANSAC result', 'full data fit' , 'RANSAC guesses', 'Location', ...
    'North');
hold off;
title('RANSAC VS full fit');
subplot(1, 2, 2);
plot(max_num_inliers_history);
title('Max num inliers over iterations');

disp('RMS of full fit =');
x = (0:0.01:1) + xstart;
disp(rms(polyval(poly, x) - polyval(full_fit, x)));
disp('RMS of RANSAC =');
x = (0:0.01:1) + xstart;
disp(rms(polyval(poly, x) - polyval(best_guess_history(:, end), x)));


%% Parts 2 and 3 - Localization with RANSAC + DLT/P3P
query_image = imread('../data/000001.png');

% Parameters from exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;

% Other parameters.
num_keypoints = 1000;

% Detect and match keypoints.
database_keypoints = keypoints;
query_harris = harris(query_image, harris_patch_size, harris_kappa);
query_keypoints = selectKeypoints(...
    query_harris, num_keypoints, nonmaximum_supression_radius);
query_descriptors = describeKeypoints(...
    query_image, query_keypoints, descriptor_radius);
database_descriptors = describeKeypoints(...
    database_image, database_keypoints, descriptor_radius);
all_matches = matchDescriptors(...
    query_descriptors, database_descriptors, match_lambda);

% Drop unmatched keypoints and get 3d landmarks for the matched ones.
matched_query_keypoints = query_keypoints(:, all_matches > 0);
corresponding_matches = all_matches(all_matches > 0);
corresponding_landmarks = p_W_landmarks(:, corresponding_matches);

% perform RANSAC to find best Pose and inliers
[R_C_W, t_C_W, inlier_mask, max_num_inliers_history, num_iteration_history] = ...
    ransacLocalization(matched_query_keypoints, corresponding_landmarks, K);

disp('Found transformation T_C_W = ');
disp([R_C_W t_C_W; zeros(1, 3) 1]);
disp('Estimated inlier ratio is');
disp(nnz(inlier_mask)/numel(inlier_mask));

matched_query_keypoints = query_keypoints(:, all_matches > 0);
corresponding_matches = all_matches(all_matches > 0);
figure(4);
semilogy(1:numel(num_iteration_history));
hold on;
semilogy(num_iteration_history);
hold off
xlabel("Iteration");
ylabel("Estimated Max Number of Iterations");

figure(5);
subplot(3, 1, 1);
imshow(query_image);
hold on;
plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
plotMatches(all_matches, query_keypoints, keypoints);
title('All keypoints and matches');

subplot(3, 1, 2);
imshow(query_image);
hold on;
plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
    matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
plot(matched_query_keypoints(2, (inlier_mask)>0), ...
    matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
plotMatches(corresponding_matches(inlier_mask>0), ...
    matched_query_keypoints(:, inlier_mask>0), ...
    keypoints);
hold off;
title('Inlier and outlier matches');
subplot(3, 1, 3);
plot(max_num_inliers_history);
title('Maximum inlier count over RANSAC iterations.');

 %% Part 4 - Same, for all frames

figure(6);
subplot(1, 3, 3);
scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-15 10 -10 5 -1 40]);

%tt=zeros(9,3);

lenD=length(p_W_landmarks);
for i = 0:9
    query_image = imread(sprintf('../data/%06d.png',i));
    
    % Detect and match keypoints.
    database_keypoints = keypoints;
    query_harris = harris(query_image, harris_patch_size, harris_kappa);
    query_keypoints = selectKeypoints(...
        query_harris, num_keypoints, nonmaximum_supression_radius);
    query_descriptors = describeKeypoints(...
        query_image, query_keypoints, descriptor_radius);
    database_descriptors = describeKeypoints(...
        database_image, database_keypoints, descriptor_radius);
    all_matches = matchDescriptors(...
        query_descriptors, database_descriptors, match_lambda);

    % Drop unmatched keypoints and get 3d landmarks for the matched ones.
    matched_query_keypoints = query_keypoints(:, all_matches > 0);
    
    %%% candidate
    if length(candidate_database_descriptors)>0
       last_u_q_p=unmatched_query_keypoints;
       last_u_q_d=unmatched_query_descriptor;   
    end
    unmatched_query_keypoints = setdiff(matched_query_keypoints,query_keypoints,'rows',"stable");
    unmatched_query_descriptors=query_descriptor(:, all_matches == 0);
    if length(candidate_database_descriptors)>0
        c_q_descriptors=union(candidate_database_descriptors,unmatched_query_descriptors,'stable');
        c_q_keypoints=union(candidate_database_keypoints,unmatched_query_keypoints,'stable');
        c_q_rot=union(candidate_database_rot,quaternion(zeros(length(unmatched_query_keypoints),4)),'stable');
        c_q_px=union(
    candidate_matches=matchDescriptors(unmatched_query_descriptors,c_q_descriptors,match_lambda);
    corresponding_cadidates= candidate_matches(candidate_matches> 0);
    corresponding_candidates_keypoints=c_q_keypoints(:, corresponding_candidates);
    corresponding_candidates_descriptors=c_q_descriptors(:, corresponding_candidates);
    corresponding_candidates_rot=c_q_rot(:, corresponding_candidates);
    corresponding_candidates_rot(:,corresponding_candidates_rot~=0)=quaternion(R_C_W,'rotmat','frame');
    end
    %%%
    corresponding_matches = all_matches(all_matches > 0);
    corresponding_landmarks = p_W_landmarks(:, corresponding_matches);

    % perform RANSAC to find best Pose and inliers
    [R_C_W, t_C_W, inlier_mask, max_num_inliers_history, ~] = ...
        ransacLocalization(matched_query_keypoints, corresponding_landmarks, K);

    matched_query_keypoints = query_keypoints(:, all_matches > 0);
    corresponding_matches = all_matches(all_matches > 0);

    % Distinguish success from failure.
    if (numel(R_C_W) > 0)
        subplot(1, 3, 3);
        orig=-R_C_W'*t_C_W;
        plotCoordinateFrame(R_C_W', orig, 2);
        disp(['Frame ' num2str(i) ' localized with ' ...
            num2str(nnz(inlier_mask)) ' inliers!']);
        view(0,0);
        %tt(i+1,:)=t_C_W'
        
    %%% rule of thumb
    % norm(position-point)/norm(sum(t_C_W)
        
        avgD=0;
        for i=1:length(p_W_landmarks)
            D=norm(p_W_landmarks(:,i)+t_C_W');
            avgD=avgD+D;
        end
        keyF=norm(t_C_W)/(avgD/lenD)
        
    %%%
    
    else
        disp(['Frame ' num2str(i) ' failed to localize!']);
          
    end
    

    
    
  
    subplot(1, 3, [1 2]);
    imshow(query_image);
    hold on;
    plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
        matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
    if (nnz(inlier_mask) > 0)
        plot(matched_query_keypoints(2, (inlier_mask)>0), ...
            matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
    end
    plotMatches(corresponding_matches(inlier_mask>0), ...
        matched_query_keypoints(:, inlier_mask>0), ...
        keypoints);
    hold off;
    %matched_query_keypoints(:, (inlier_mask)>0)
    title('Inlier and outlier matches');
    pause(0.01);
end
% avgD=0;
% lenD=length(p_W_landmarks);
% for i=1:length(p_W_landmarks)
%  D=norm(p_W_landmarks(:,i)-[0.2812,-0.0756,-6.7810]);
% avgD=avgD+D;
% end
% norm(t_C_W)/(avgD/lenD)