%% 🚀 Mission Control Center
% 여기서 모든 설정을 바꾸고 실행하세요!
clear; clc; close all;

%% 1. 사용자 설정 (User Settings)
% ---------------------------------------------------------
N = 80;           % 타임 스텝
tf = 100.0;        % 총 비행 시간

% 맵 크기 [x, y]
room_min = [-5, -5];
room_max = [ 5,  5];

% 장애물 목록: [x, y, rx, ry]
% (타원: rx=가로반지름, ry=세로반지름)
obstacles = [
    0.7,  0.7, 0.25, 0.10;
    0.1,  0.0, 0.10, 0.25;
    0.0, -0.5, 0.17, 0.17;
   -0.5,  1.0, 0.10, 0.10
];

% 시작 및 목표 상태 [x, y, theta, vx, vy, omega]
start_state = [-4.9, -4.9, 0.0, 0.0, 0.0, 0.0];
goal_state  = [ 4.9,  4.9, 0.0, 0.0, 0.0, 0.0];

% ---------------------------------------------------------

%% 2. 맵 띄우기 & 경유지 클릭
r_robot = 0.215; clearance = 0.05;

figure(1); clf; hold on; axis equal; grid on;
xlim([room_min(1), room_max(1)]);
ylim([room_min(2), room_max(2)]);
title('Click Waypoints! (Right-click or Enter to Finish)');
xlabel('x [m]'); ylabel('y [m]');

% 장애물 그리기
t = linspace(0, 2*pi, 100);
for i = 1:size(obstacles, 1)
    ox = obstacles(i,1); oy = obstacles(i,2);
    rx = obstacles(i,3); ry = obstacles(i,4);
    fill(ox + rx*cos(t), oy + ry*sin(t), [0.5 0.5 0.5], 'EdgeColor', 'k');
    plot(ox + (rx+r_robot+clearance)*cos(t), oy + (ry+r_robot+clearance)*sin(t), 'r--');
end

% 시작/목표 표시
plot(start_state(1), start_state(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goal_state(1), goal_state(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

% 마우스 클릭으로 경유지 수집
disp('>>> 맵에서 경유지를 클릭하세요. (종료: Enter)');
waypoints = [];
while true
    [x, y, button] = ginput(1);
    if isempty(button) || button == 3, break; end
    plot(x, y, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    text(x, y, sprintf('  WP%d', size(waypoints, 1)+1));
    waypoints = [waypoints; x, y];
end

%% 3. Julia용 설정 파일 저장 (CSV)
disp('>>> 설정을 CSV 파일로 저장합니다...');

% (1) 기본 파라미터 (N, tf)
writematrix([N; tf], 'config_params.csv');

% (2) 방 크기 (min, max)
writematrix([room_min; room_max], 'config_room.csv');

% (3) 장애물
writematrix(obstacles, 'config_obstacles.csv');

% (4) 상태 (Start, Goal)
writematrix([start_state; goal_state], 'config_state.csv');

% (5) 경유지 (없으면 빈 파일)
if isempty(waypoints)
    writematrix("empty", 'config_waypoints.csv'); % 빈 파일 표시
else
    writematrix(waypoints, 'config_waypoints.csv');
end

fprintf('>>> 설정 저장 완료! 이제 Julia를 실행하세요.\n');