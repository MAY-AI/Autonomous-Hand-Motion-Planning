%% Gabriel Agostine
% ASEN 5254
% Final Project

% Hand and Arm Visualizer

clear; clc; close all force;
% Clears workspace

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS'); %#ok<TNOW1,DATST>
filename = sprintf('hand_motion_%s.mp4', timestamp);

v = VideoWriter(filename, 'MPEG-4');
v.FrameRate = 120;
open(v);

% Read all joint position files - now including ring and little fingers
joints = {'Elbow', 'Wrist', ...
          'BaseThumb', 'BaseIndex', 'BaseMiddle', 'BaseRing', 'BaseLittle', ...
          'KnuckleThumb', 'KnuckleIndex', 'KnuckleMiddle', 'KnuckleRing', 'KnuckleLittle', ...
          'MiddleThumb', 'MiddleIndex', 'MiddleMiddle', 'MiddleRing', 'MiddleLittle', ...
          'TipThumb', 'TipIndex', 'TipMiddle', 'TipRing', 'TipLittle'};

% Initialize cell array to store joint data
jointData = cell(length(joints), 1);

% Read CSV files
for i = 1:length(joints)
    filename = sprintf('Joint_%d_Movement.csv', i-1);
    jointData{i} = readmatrix(filename);
end

% Create figure
fig = figure;

% Animation loop
for t = 1:size(jointData{1}, 1)
    clf;
    hold on;
    grid on;
    set(gca, 'XDir', 'reverse');
    set(gca, 'ZDir', 'reverse');
    view(3);
    
    % Plot arm segments
    plot3([0, jointData{1}(t,1)], [0, jointData{1}(t,2)], [0, jointData{1}(t,3)], 'k-', 'LineWidth', 2);  % Shoulder to elbow
    plot3([jointData{1}(t,1), jointData{2}(t,1)], [jointData{1}(t,2), jointData{2}(t,2)], ...
          [jointData{1}(t,3), jointData{2}(t,3)], 'k-', 'LineWidth', 2);  % Elbow to wrist
    
    % Plot fingers
    % Thumb joints (3,8,13,18)
    plotFinger(jointData, t, [3,8,13,18], 'r');
    % Index joints (4,9,14,19)
    plotFinger(jointData, t, [4,9,14,19], 'g');
    % Middle joints (5,10,15,20)
    plotFinger(jointData, t, [5,10,15,20], 'b');
    % Ring joints (6,11,16,21)
    plotFinger(jointData, t, [6,11,16,21], 'm');
    % Little joints (7,12,17,22)
    plotFinger(jointData, t, [7,12,17,22], 'c');
    
    % Set axis properties
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Frame %d', t));
    axis equal;
    
    % Pause to create animation
    pause(0.01);
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

% Helper function to plot finger segments
function plotFinger(jointData, t, indices, color)
    % Plot lines connecting finger joints
    for i = 1:length(indices)-1
        plot3([jointData{indices(i)}(t,1), jointData{indices(i+1)}(t,1)], ...
              [jointData{indices(i)}(t,2), jointData{indices(i+1)}(t,2)], ...
              [jointData{indices(i)}(t,3), jointData{indices(i+1)}(t,3)], ...
              '-', 'Color', color, 'LineWidth', 1.5);
    end
    
    % Plot joints as spheres
    for i = 1:length(indices)
        scatter3(jointData{indices(i)}(t,1), jointData{indices(i)}(t,2), ...
                jointData{indices(i)}(t,3), 50, color, 'filled');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%