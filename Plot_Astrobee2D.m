%% 1. 데이터 불러오기
traj_data = readtable('astrobee_result.csv');
config_data = readtable('astrobee_config.csv'); 
room_data = readtable('astrobee_room.csv'); % [추가] 방 크기 정보 로드

%% 2. 로봇 물리 상수
r_robot = 0.215;
clearance = 0.05;

%% 3. 그래프 그리기
h = figure(1); clf;
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Astrobee SE2 Trajectory (Auto-Sized)');

% [수정] 고정값(-5, 5) 대신 읽어온 방 크기로 설정
xlim([room_data.min_x, room_data.max_x]);
ylim([room_data.min_y, room_data.max_y]);

hold on;

% (1) 타원형 장애물 그리기
t = linspace(0, 2*pi, 100);

for i = 1:height(config_data)
    ox = config_data.ox(i);
    oy = config_data.oy(i);
    
    % [수정] rx, ry 따로 읽기
    rx = config_data.rx(i);
    ry = config_data.ry(i);
    
    % A. 실제 장애물 (타원)
    x_obs = ox + rx * cos(t);
    y_obs = oy + ry * sin(t);
    fill(x_obs, y_obs, [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    
    % B. 안전 경계선 (타원 팽창)
    % 가로/세로 각각 늘려줌
    rx_safe = rx + r_robot + clearance;
    ry_safe = ry + r_robot + clearance;
    
    x_safe = ox + rx_safe * cos(t);
    y_safe = oy + ry_safe * sin(t);
    plot(x_safe, y_safe, 'r--', 'LineWidth', 1);
end

% (2) 궤적 및 로봇 그리기
plot(traj_data.rx, traj_data.ry, 'k:', 'LineWidth', 1); 

% ... (이후 애니메이션 코드는 원형 로봇으로 그리면 됩니다) ...