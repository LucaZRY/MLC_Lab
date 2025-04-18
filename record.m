% 提取数据
try
    % 检查 logsout 是否存在
    if exist('logsout', 'var') == 0
        error('logsout 变量未找到。请确保已启用 Signal Logging 并运行仿真。');
    end
    
    % 提取每个信号的数据
    yaw_pos = logsout.get('yaw_position').Values;
    pitch_pos = logsout.get('pitch_position').Values;
    yaw_volt = logsout.get('yaw_voltage').Values;
    pitch_volt = logsout.get('pitch_voltage').Values;
    
    % 统一时间向量（假设所有信号时间戳一致）
    time = yaw_pos.Time;
    
catch ME
    disp('数据提取失败，请检查信号名称或日志配置：');
    disp(ME.message);
    return;
end

% 绘制所有信号曲线
figure('Name', 'Simulink 记录数据', 'Position', [100 100 1200 800]);

% 俯仰位置
subplot(2,2,1);
plot(time, pitch_pos.Data, 'b', 'LineWidth', 1.5);
title('Pitch Position (rad)');
xlabel('Time (s)');
grid on;

% 偏航位置
subplot(2,2,2);
plot(time, yaw_pos.Data, 'r', 'LineWidth', 1.5);
title('Yaw Position (rad)');
xlabel('Time (s)');
grid on;

% 俯仰电压
subplot(2,2,3);
plot(time, pitch_volt.Data, 'g', 'LineWidth', 1.5);
title('Pitch Voltage (V)');
xlabel('Time (s)');
ylim([-30 30]);  % 假设电压限制在 ±25V
grid on;

% 偏航电压
subplot(2,2,4);
plot(time, yaw_volt.Data, 'm', 'LineWidth', 1.5);
title('Yaw Voltage (V)');
xlabel('Time (s)');
ylim([-30 30]);
grid on;

% 保存数据到文件
data_table = table(time, pitch_pos.Data, yaw_pos.Data, pitch_volt.Data, yaw_volt.Data, ...
    'VariableNames', {'Time', 'PitchPosition', 'YawPosition', 'PitchVoltage', 'YawVoltage'});

% 保存为 Excel 文件
writetable(data_table, 'simulation_data.xlsx');
disp('数据已保存为 simulation_data.xlsx');

% 保存为 MAT 文件
save('simulation_data.mat', 'data_table');
disp('数据已保存为 simulation_data.mat');