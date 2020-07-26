%% 
% MXMotorSample.m
% B. Mathie 2/11/2020
% 

%%


%%
global lib_name
lib_name            = 'dxl_x64_c';
com_port_name        = 'COM4';  



if libisloaded(lib_name)
% Close port
    closePort(port_num);
end
% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end


%% Set up a global for the com port


%% NEED TO ADDRESS THE USE OF GLOBALS PERHAPS JUST DEFINE com.UserData(4)=com
%% Set up other convenient globals  Eventually delete or replace them
global FF
global oldangvel
global inlast;
global predang
global actualang
global pvel
global actualvel
global SS
global port_num
global Fff


%FF=[0 0 0];
%FF=double(FF);
oldangvel=[pi 0];
predang=[];
actualang=[];
pvel=[];
actualvel=[];
SS=0;


%% Initialize variables that will be passed to the timer fcn via its
% UserData property
count=0;
time=0;


%% Initialization Pro Motor Parameters & Variables
DXL1_ID                         = 1;            % Dynamixel#1 ID: 1
DXL2_ID                         = 2;            % Dynamixel#2 ID: 2
DXL3_ID                         = 3;
baudrate                        = 2000000;
port_num                        = portHandler(com_port_name);

ADDR_TORQUE_ENABLE              = 64;                % Control table address is different in Dynamixel model
ADDR_GOAL_POSITION              = 116;
ADDR_PRESENT_POSITION           = 132;
ADDR_VEL_PROFILE                = 112;
ADDR_PRESENT_VEL                = 128;


PROTOCOL_VERSION                = 2.0;

VELOCITY_1                      = 100; 
VELOCITY_2                      = 100;

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;




%% Initialization of MX Motors  
calllib(lib_name, 'setBaudRate', port_num, baudrate);
packetHandler(); % Initialize PacketHandler Structs




if (openPort(port_num)) % Open port
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end




    
    
    
    
    
%% Torque enable =0 Set Position Mode then Torque Enable=1 uses 1 byte hence write1ByteTxRx()
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, 0); % Torque enable set to 0  Motor 1
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, 0);% Torque enable set to 0  Motor 2

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 11, 3); % Set both motors to Position Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 11, 3);

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, 1); % Enable Dynamixel#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, 1); % Enable Dynamixel#2 Torque

%% Set PID values for each motor uses 2 bytes hence write2ByteTxRx
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 80, 1000 ); % Motor 1 D
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 82, 50); % Motor 1 I
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 84, 1000); %Motor 1 P

write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 80, 1000 ); % Motor 2 D
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 82, 50); % Motor 2 I
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 84, 1000); %Motor 2 P

%% Move Motor to new position  (uses 4 bytes hence write4ByteTxRx()

% Set velocity
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID,ADDR_VEL_PROFILE, 10); % Velocity is set to a low value to move motor slowly
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_VEL_PROFILE, 10); 

% Command motors to move 
servo_position = 2000;
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_GOAL_POSITION, servo_position); %% ENTER YOUR NEW POSITION HERE
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_GOAL_POSITION, servo_position);

 %% =======================algorithm=======================================
 % make sure to load data to our variables 
% performs fast fourier transform 
Fs = 1000; % Sampling frequency
T = 1/Fs; % period
L = length(data); % lenght of signal 
t = (0:L -1)*T; % time vector in seconds 
t= t'; % transpose 

% graph the RMS data
plot(t, data(:,3))
xlabel('Seconds');
ylabel('Milivolts (mV)');
title('Electromyography Signal')
shg % when we run the graph pops up automatically

movement = 2000; % variable to set servo position
% code below is used to set the initial position of the arm 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_GOAL_POSITION, movement); % servo 1 set position 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_GOAL_POSITION, movement); % servo 2 set position
i = 1; % used to index our EMG data
 while (i < length(data)) % stop while loop once i index reached end of data
    
    if data(i,3) >= 0.049 % threshold to move robotic arm up
        movement = movement + 5; % step size to increase arm position by 5
        if movement > 2000 % used to make sure we don't pass maximum range of arm
            movement = 2000;
        end 
         
        fprintf('movement up = %f', movement)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_GOAL_POSITION, movement); 
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_GOAL_POSITION, movement); 
    else % movement to move arm down
        if movement < 1600 % used to make sure the arm does not pass minimum height
            movement = 1600;
        end 
        movement = movement - 5; % step size to decrease arm position by 5
        fprintf('movement down  = %f', movement)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_GOAL_POSITION, movement);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_GOAL_POSITION, movement);
    end 
    
    i = i + 40; % index through EMG data by 40 points at a time
    disp(data(i,3)) % display EMG data
 end

 %%
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_GOAL_POSITION, 2500); %% ENTER YOUR NEW POSITION HERE
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_GOAL_POSITION, 2500);
%% Torque enable =0 Set Position Mode then Torque Enable=1 uses 1 byte hence write1ByteTxRx()
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, 0); % Torque enable set to 0  Motor 1
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, 0);% Torque enable set to 0  Motor 2

% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, 11, 3); % Set both motors to Position Mode
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, 11, 3);

% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, 1); % Enable Dynamixel#1 Torque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, 1); % Enable Dynamixel#2 Torque


disp('Finished')

 closePort(port_num);

 

