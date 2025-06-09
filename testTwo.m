clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%{
********* DYNAMIXEL Model *********
***** XM430-W350 Configuration *****
%}

My_DXL = 'X_SERIES'; % XM430-W350 is part of X-Series

% Control table addresses for X-Series (XM430-W350)
ADDR_OPERATING_MODE         = 11;
ADDR_TORQUE_ENABLE          = 64;
ADDR_GOAL_CURRENT           = 102;          % Goal Current for current-based position control
ADDR_GOAL_POSITION          = 116;
ADDR_PRESENT_CURRENT        = 126;          % Present Current
ADDR_PRESENT_POSITION       = 132;

% Position limits for safety (adjust these based on your hardware constraints)
DXL_MINIMUM_POSITION_VALUE  = 1024;        % Adjust these values to prevent hardware collision
DXL_MAXIMUM_POSITION_VALUE  = 3072;        % These are example values - customize for your setup
BAUDRATE                    = 115200;

% DYNAMIXEL Protocol Version
PROTOCOL_VERSION            = 2.0;          

% Factory default ID of DYNAMIXEL
DXL_ID                      = 11; 

% Device name (adjust for your system)
DEVICENAME                  = '/dev/ttyUSB0';       

% Control Table Values
OPERATING_MODE = 5;        % Current-based position control mode
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

% Current control parameters (adjust based on your needs)
MAX_CURRENT_LIMIT           = 200;          % Maximum current in mA (adjust for your motor)
GOAL_CURRENT_VALUES         = [100, -100]; % Current values to alternate between (mA)

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler and PacketHandler
port_num = portHandler(DEVICENAME);
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];
dxl_goal_current = GOAL_CURRENT_VALUES(index);

dxl_error = 0;
dxl_present_position = 0;
dxl_present_current = 0;

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Disable torque to change operating mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Set operating mode to Current-based Position Control (5)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Operating mode set to: %d \n', OPERATING_MODE);
end

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected\n');
end

fprintf('- Goal Current will be set to apply torque\n');
fprintf('- Position limits: %d to %d (safety bounds)\n', DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
fprintf('- Motor will stop if it reaches position limits\n\n');

while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Set goal position (acts as position limit)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    % Set goal current (this applies the torque)
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, typecast(int16(GOAL_CURRENT_VALUES(index)), 'uint16'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    fprintf('Goal Position: %d, Goal Current: %d mA\n', dxl_goal_position(index), GOAL_CURRENT_VALUES(index));

    % Monitor the motor for a few seconds
    for i = 1:50  % Monitor for about 5 seconds (100ms intervals)
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        % Read present current
        dxl_present_current = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_CURRENT);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] GoalPos:%04d PresPos:%04d GoalCur:%04d mA PresCur:%04d mA\n', ...
                DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_present_position), 'int32'), ...
                GOAL_CURRENT_VALUES(index), typecast(uint16(dxl_present_current), 'int16'));

        % Check if position is within safe bounds
        current_pos = typecast(uint32(dxl_present_position), 'int32');
        if current_pos <= DXL_MINIMUM_POSITION_VALUE || current_pos >= DXL_MAXIMUM_POSITION_VALUE
            fprintf('WARNING: Motor reached position limit! Stopping current command.\n');
            % Set current to 0 to stop applying torque
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, 0);
            break;
        end

        pause(0.1); % 100ms delay
    end

    % Set current to 0 before changing direction
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, 0);
    pause(0.5); % Wait a bit before changing direction

    % Change goal position and current direction
    if index == 1
        index = 2;
    else
        index = 1;
    end
end

% Set current to 0 before disabling torque
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_CURRENT, 0);
pause(0.5);

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;