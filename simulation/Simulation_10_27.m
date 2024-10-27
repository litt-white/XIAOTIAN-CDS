clc;
clear;

% --------------------- 初始化参数 --------------------- 
frequency = 10e9; % 雷达工作频率 10 GHz
lambda = physconst('LightSpeed') / frequency; % 波长
pulseWidth = 1e-6; % 脉冲宽度
prf = 1e3; % 脉冲重复频率 (PRF)
maxRange = 2000; % 最大探测距离
rangeRes = 1; % 距离分辨率
RCS = 0.1; % 无人机群的RCS假设为0.1平方米
scanRate = 360 / 3; % 雷达旋转速度, 每3秒转360°
numDrones = 10; % 无人机数量
formationSpacing = 10; % 编队间距（米）

% 波束宽度
beamwidth = 90; % 波束宽度（度）

% 多径效应
numMultipath = 3; % 多径数目
multipathDelays = [0, 0.1, 0.2]; % 传播延迟 (秒)
multipathAmplitudes = [1, 0.5, 0.3]; % 幅度衰减

% --------------------- 定义编队形态 --------------------- 
formationType = 'rectangle'; % 编队类型，可以选择'line', 'rectangle', 'triangle', 'diamond', 'V'

% 根据编队类型生成无人机初始位置
if strcmp(formationType, 'line')
    droneInitialPositions = [linspace(0, (numDrones-1) * formationSpacing, numDrones); ...
                             zeros(1, numDrones); ...
                             50 * ones(1, numDrones)]; % 无人机高度固定为50米
elseif strcmp(formationType, 'rectangle')
    rows = ceil(sqrt(numDrones)); % 计算行数
    cols = ceil(numDrones / rows); % 计算列数
    [X, Y] = meshgrid(0:formationSpacing:(cols-1) * formationSpacing, 0:formationSpacing:(rows-1) * formationSpacing);
    droneInitialPositions = [X(:)'; Y(:)'; 50 * ones(1, numel(X(:)))]; % 维度一致
elseif strcmp(formationType, 'triangle')
    droneInitialPositions = zeros(3, numDrones);
    for i = 1:numDrones
        row = floor(sqrt(i)); % 当前行
        col = i - (row * (row + 1)) / 2; % 当前列
        droneInitialPositions(1, i) = col * formationSpacing; % x坐标
        droneInitialPositions(2, i) = row * formationSpacing; % y坐标
        droneInitialPositions(3, i) = 50; % z坐标
    end
elseif strcmp(formationType, 'diamond')
    droneInitialPositions = zeros(3, numDrones);
    for i = 1:numDrones
        row = floor(sqrt(i)); % 当前行
        offset = mod(i, (row + 1)); % 当前偏移
        droneInitialPositions(1, i) = (row - offset) * formationSpacing; % x坐标
        droneInitialPositions(2, i) = (row + offset) * formationSpacing; % y坐标
        droneInitialPositions(3, i) = 50; % z坐标
    end
elseif strcmp(formationType, 'V')
    droneInitialPositions = zeros(3, numDrones);
    for i = 1:numDrones
        angle = linspace(-pi/4, pi/4, numDrones); % V字形的角度
        droneInitialPositions(1, i) = (i - (numDrones / 2)) * formationSpacing * cos(angle(i)); % x坐标
        droneInitialPositions(2, i) = (i - (numDrones / 2)) * formationSpacing * sin(angle(i)); % y坐标
        droneInitialPositions(3, i) = 50; % z坐标
    end
else
    error('Unsupported formation type');
end

% 定义无人机的巡航速度 (vx, vy, vz)，单位为米/秒
droneVelocities = [20 * randn(numDrones, 1), ...
                   20 * randn(numDrones, 1), ...
                   zeros(numDrones, 1)]; % 无人机沿x, y方向巡航，不在z方向移动

% --------------------- 创建雷达平台 --------------------- 
radarPlatform = phased.Platform('InitialPosition', [0; 0; 10], ...
                                'Velocity', [0; 0; 0]);

% --------------------- 创建雷达和目标 --------------------- 
transmitter = phased.Transmitter('PeakPower', 1e3, ...
                                 'Gain', 40);
receiver = phased.ReceiverPreamp('Gain', 40, ...
                                 'NoiseFigure', 2);

dronesRCS = RCS * ones(1, numDrones); % 每个无人机的RCS
droneTargets = phased.RadarTarget('MeanRCS', dronesRCS, ...
                                  'OperatingFrequency', frequency);

% --------------------- 仿真雷达扫描和探测 --------------------- 
rotateRound = 50; % 定义扫描角度圈数
azimuthAngles = 0:scanRate:360 * rotateRound;

% 创建结果表格
resultTable = table([], [], [], [], [], [], [], ...
                    'VariableNames', {'Time', 'SlantRange', 'RadialVelocity', ...
                                      'AzimuthAngle', 'ElevationAngle', 'Round', 'ClutterLabel'});

% 创建集群结果表格
clusterResultTable = table([], [], [], [], [], [], [], ...
                            'VariableNames', {'Time', 'ClusterCenterX', 'ClusterCenterY', 'ClusterCenterZ', ...
                                              'AverageVelocityX', 'AverageVelocityY', 'AverageVelocityZ'});

% 计算每个方位角更新的时间步长
rotationTimePerStep = 3 / (360 / scanRate); % 每个方位角更新的时间 (秒)

% 模拟雷达扫描
simTime = 0; % 初始时间
for az = azimuthAngles
    simTime = simTime + rotationTimePerStep;

    % 更新无人机位置
    for i = 1:numDrones
        droneInitialPositions(:, i) = droneInitialPositions(:, i) + droneVelocities(i, :)' * rotationTimePerStep; % 更新位置
    end

    % 计算集群中心和速度
    clusterCenter = mean(droneInitialPositions, 2); % 计算集群中心
    averageVelocity = mean(droneVelocities, 1)'; % 计算集群速度，转置为列向量

    % 添加集群数据行
    clusterRow = {simTime, clusterCenter(1), clusterCenter(2), clusterCenter(3), ...
                  averageVelocity(1), averageVelocity(2), averageVelocity(3)}; 
    clusterResultTable = [clusterResultTable; clusterRow]; % 追加新行

    % 记录每个无人机数据
    for i = 1:numDrones
        [range, angle] = rangeangle(droneInitialPositions(:, i), radarPlatform.InitialPosition);
        radialVelocity = radialspeed(droneInitialPositions(:, i), droneVelocities(i, :)', radarPlatform.InitialPosition, [0; 0; 0]);
        azimuthAngle = angle; % 方位角
        elevationAngle = atan2(droneInitialPositions(3, i) - radarPlatform.InitialPosition(3), range); % 俯仰角

        % 考虑遮蔽效应：检查是否有其他无人机在此无人机与雷达之间
        obstructed = false;
        for j = 1:numDrones
            if j ~= i
                % 计算无人机i与无人机j之间的遮挡
                if norm(droneInitialPositions(:, i) - droneInitialPositions(:, j)) < formationSpacing && ...
                   droneInitialPositions(3, j) < droneInitialPositions(3, i) % j在i的前面并且高度低
                    obstructed = true;
                    break;
                end
            end
        end

        % 考虑波束宽度
        if any(abs(azimuthAngle) <= beamwidth / 2) && ~obstructed
            % 计算多径效应
            multipathSignal = 0;
            for m = 1:numMultipath
                delayedRange = range + multipathDelays(m) * physconst('LightSpeed'); % 增加传播延迟
                multipathSignal = multipathSignal + (multipathAmplitudes(m) / delayedRange); % 简化的幅度模型
            end
            
            % 添加无人机数据行
            newRow = {simTime, range + multipathSignal, radialVelocity, azimuthAngle, elevationAngle, floor(az/360) + 1, 1};
            resultTable = [resultTable; newRow]; % 追加新行
        end
    end

    % 生成杂波（云层、地面反射等）
    numClutter = randi([10, 20]); % 随机生成10到20个杂波
    for j = 1:numClutter
        clutterPos = [rand() * 2000; rand() * 2000; 0]; % 随机生成杂波位置
        [clutterRange, clutterAngle] = rangeangle(clutterPos, radarPlatform.InitialPosition);
        clutterElevationAngle = atan2(clutterPos(3) - radarPlatform.InitialPosition(3), clutterRange);

        % 生成高斯杂波
        gaussianClutterRadialVelocity = normrnd(0, 10); % 平均0，标准差10的高斯分布
        % 添加高斯杂波数据行
        gaussianClutterRow = {simTime, clutterRange, gaussianClutterRadialVelocity, clutterAngle, clutterElevationAngle, floor(az/360) + 1, 0};
        gaussianClutterRowTable = cell2table(gaussianClutterRow, 'VariableNames', resultTable.Properties.VariableNames);
        resultTable = [resultTable; gaussianClutterRowTable]; % 追加高斯杂波数据行

        % 生成莱斯杂波
        ricianClutterRadialVelocity = raylrnd(10); % 平均值10的莱斯分布
        % 添加莱斯杂波数据行
        ricianClutterRow = {simTime, clutterRange, ricianClutterRadialVelocity, clutterAngle, clutterElevationAngle, floor(az/360) + 1, 0};
        ricianClutterRowTable = cell2table(ricianClutterRow, 'VariableNames', resultTable.Properties.VariableNames);
        resultTable = [resultTable; ricianClutterRowTable]; % 追加莱斯杂波数据行

        % 生成对数—正态杂波
        lognormalClutterRadialVelocity = lognrnd(0, 0.5); % 平均0，标准差0.5的对数正态分布
        % 添加对数—正态杂波数据行
        lognormalClutterRow = {simTime, clutterRange, lognormalClutterRadialVelocity, clutterAngle, clutterElevationAngle, floor(az/360) + 1, 0};
        lognormalClutterRowTable = cell2table(lognormalClutterRow, 'VariableNames', resultTable.Properties.VariableNames);
        resultTable = [resultTable; lognormalClutterRowTable]; % 追加对数—正态杂波数据行
    end
end

% 输出仿真结果
disp(resultTable);
disp(clusterResultTable);

% 将表格数据保存到文件
writetable(resultTable, 'RadarSimulationResults.csv');
writetable(clusterResultTable, 'ClusterResults.csv');
