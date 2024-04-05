vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

Ts = 0.05; %0.01 s = 10 ms %default simulation dt on vrep is 50 ms
w_hdl = zeros(2,1);
v_w = ones(2,1).*60*Ts; %degree/s convert to nominal/Ts
ultsnd_h = zeros(3,1);
ultsnd_masure_Zaxs = zeros(3,1);

if (clientID>-1)
    disp('Connected to remote API server')
    
    %------getting objects handlers------
    %get motor handler
    [returnCode,w_hdl(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,w_hdl(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    %get ultrasound handler
    [returnCode,ultsnd_h(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [returnCode,ultsnd_h(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking); %the angle of this sensor is little bit off
    [returnCode,ultsnd_h(3)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking);
    
    n=0;
    while n<200
        %-----getting data-----
        %get robot pos and orientation
        [returnCode,rob_poss]=vrep.simxGetObjectPosition(clientID,rob_h,-1,vrep.simx_opmode_blocking); %ret in meter, vecor
        [returnCode,rob_ortt]=vrep.simxGetObjectOrientation(clientID,rob_h,-1,vrep.simx_opmode_blocking); %ret in radian, vector
        %get sensor data
        detectedPoint={};
        detectionState=zeros(3,1);
        for i=1:3
            [returnCode,detectionState(i),detectedPoint{i},detectedObjectHandle,detectedSurfaceNormalVector]=vrep.simxReadProximitySensor(clientID,ultsnd_h(i),vrep.simx_opmode_blocking);
        end
        %-----processing data-----
        % sensor value constrain from 0 cm to 400 cm
        for i=1:3
            if detectionState(i)
                ultsnd_masure_Zaxs(i) = detectedPoint{i}(3) *100; % ret in centimeter, vector
            else
                ultsnd_masure_Zaxs(i) = 400;
            end
        end
        disp(ultsnd_masure_Zaxs);
        
        
        %-----Controller START here-----
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-----Controller END here-------
        
        
        %-----Actuate motors-----
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(1),0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(2),0,vrep.simx_opmode_blocking);
        %increamnet counter
        n=n+1;
        %BREAK LOOP BY BUTTON
        ButtonHandle = uicontrol('Style', 'PushButton', ...
                                 'String', 'Stop loop', ...
                                 'Callback', 'delete(gcbf)');
        drawnow;
        if ~ishandle(ButtonHandle)
            disp('Loop stopped by user');
            break;
        end
    end
    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end

[returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(1),0,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(2),0,vrep.simx_opmode_blocking);

vrep.delete(); % call the destructor!

disp('Program ended');