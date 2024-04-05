disp('Program started');

%TFLC RULES
rules_TFLC_L = [
    1 1 1 1 3 3 4;
    2 2 1 2 3 5 6;
    2 2 2 3 5 6 7;
    1 1 1 4 6 6 7;
    2 2 3 5 5 6 7;
    1 1 4 6 5 6 7;
    2 2 3 7 5 6 7
    ];
rules_TFLC_L_var = {'z'; 's'; 'nm'; 'm'; 'nh'; 'h'; 'vh'};
rules_TFLC_R = flip(rules_TFLC_L,2);

%OAFLC RULES
rules_OAFLC_R_L = [
    1 1;
    2 1;
    2 1;
    1 1;
    2 1;
    2 1;
    1 1;
    2 1;
    2 1;
    1 2;
    1 1;
    5 3;
    3 5;
    5 3;
    5 3;
    1 2;
    5 3;
    5 3;
    1 2;
    3 5;
    1 1;
    1 2;
    3 5;
    5 3;
    1 2;
    3 5;
    4 4;
    ];
var_OAFLC_output={'nh', 'n', 'p', 'hp', 'vhp'};

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
    
    %get motor handler
    [returnCode,w_hdl(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,w_hdl(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    %get ultrasound handler
    [returnCode,ultsnd_h(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [returnCode,ultsnd_h(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking); %the angle of this sensor is little bit off
    [returnCode,ultsnd_h(3)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking);
    %get robot and GOAL handler
    [returnCode,rob_h]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [returnCode,gol_h]=vrep.simxGetObjectHandle(clientID,'GOAL',vrep.simx_opmode_blocking);
    
    n=0;
    while n<200
        %-----getting data-----
        %get robot pos and orientation
        [returnCode,rob_poss]=vrep.simxGetObjectPosition(clientID,rob_h,-1,vrep.simx_opmode_blocking); %ret in meter, vecor
        [returnCode,rob_ortt]=vrep.simxGetObjectOrientation(clientID,rob_h,-1,vrep.simx_opmode_blocking); %ret in radian, vector
        %get goal pos and orientation
        [returnCode,gol_poss]=vrep.simxGetObjectPosition(clientID,gol_h,-1,vrep.simx_opmode_blocking); %ret in meter, vecor
        [returnCode,gol_ortt]=vrep.simxGetObjectOrientation(clientID,gol_h,-1,vrep.simx_opmode_blocking); %ret in radian, vecor
        %get sensor data
        detectedPoint={};
        detectionState=zeros(3,1);
        for i=1:3
            [returnCode,detectionState(i),detectedPoint{i},detectedObjectHandle,detectedSurfaceNormalVector]=vrep.simxReadProximitySensor(clientID,ultsnd_h(i),vrep.simx_opmode_blocking);
        end
        %-----processing data-----
        %position goal error / vector magnitude, and orientation error
        %position error
        vec_pos = gol_poss-rob_poss;
        pos_err = sqrt(sum((vec_pos).^2)); %in meter
        %orientation error
        gamma_r = rob_ortt(3);
        gamma_g = gol_ortt(3);
        rob_x_2_g_or = [cos(gamma_r) sin(gamma_r); -sin(gamma_r) cos(gamma_r)]*[cos(gamma_g) -sin(gamma_g); sin(gamma_g) cos(gamma_g)]*[1;0];
        orr_err = atan2(rob_x_2_g_or(2), rob_x_2_g_or(1));
        orr_err = rad2deg(orr_err);
        %Goal position relative to robot position to find heading error
        fTr = [cos(gamma_r) -sin(gamma_r) rob_poss(1); sin(gamma_r) cos(gamma_r) rob_poss(2); 0 0 1];
        gol_2_r_pos = (fTr)\[gol_poss(1); gol_poss(2); 1];
        %heading error
        hed_err = atan2(gol_2_r_pos(2), gol_2_r_pos(1));
        hed_err = rad2deg(hed_err);
        % sensor value constrain from 0 cm to 400 cm
        for i=1:3
            if detectionState(i)
                ultsnd_masure_Zaxs(i) = detectedPoint{i}(3) *100; % ret in centimeter, vector
            else
                ultsnd_masure_Zaxs(i) = 400;
            end
        end
        %-----Controller START here-----
        if min(ultsnd_masure_Zaxs)<60 %OBSTACLE DETECTED, OAFLC control
            %----------OAFLC control
            %fuzzyfication
            fuz_s1 = check_obstacle_membership(ultsnd_masure_Zaxs(1)); %left sensor
            fuz_s2 = check_obstacle_membership(ultsnd_masure_Zaxs(2)); %front sensor
            fuz_s3 = check_obstacle_membership(ultsnd_masure_Zaxs(3)); %right sensor
            %inference
            [mat_rules_val,v_OAFLC_L_th, v_OAFLC_R_th] = relate_OAFLC (fuz_s1, fuz_s2, fuz_s3, rules_OAFLC_R_L);
            %calculating centroid OAFLC R
            dt = 0.01;
            t = -10:dt:15; %output data range
            out = [];
            for i=1:length(t)
                ret=check_vl_vr_oaflc_membership(t(i)); %output fuzzification
                ret = min(v_OAFLC_R_th, ret);
                %Max value / real value of the inference
                out = [out max(ret)]; %MAX
            end
            centeroid_OAFLC_R = sum(out.*t)/sum(out)
            %calculating centroid OAFLC L
            out = [];
            for i=1:length(t)
                ret=check_vl_vr_oaflc_membership(t(i)); %output fuzzification
                ret = min(v_OAFLC_L_th, ret);
                %Max value / real value of the inference
                out = [out max(ret)]; %MAX
            end
            centeroid_OAFLC_L = sum(out.*t)/sum(out)
            %actuate the wheel
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(1),centeroid_OAFLC_L,vrep.simx_opmode_blocking);%left
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(2),centeroid_OAFLC_R,vrep.simx_opmode_blocking);%right
        else %obstacle NOT detected, TFLC control
            %----------TFLC control
            %fuzzyfication
            fuz_dist = check_distance_membership(pos_err);
            fuz_orie = check_err_angle_membership(hed_err);
            %inference
            [mat_TFLC_L_rel,v_TFLC_L_th] = relate_TFLC (fuz_dist, fuz_orie, rules_TFLC_L);
            [mat_TFLC_R_rel,v_TFLC_R_th] = relate_TFLC (fuz_dist, fuz_orie, rules_TFLC_R);
            %calculating centroid TFLC L
            dt = 0.01;
            t = -1:dt:13; %output data range
            out = [];
            for i=1:length(t)
                ret=check_velocity_membership(t(i)); %output fuzzification
                ret = min(v_TFLC_L_th, ret);
                %Max value / real value of the inference
                out = [out max(ret)]; %MAX
            end
            centeroid_TFLC_L = sum(out.*t)/sum(out);
            %calculating centroid TFLC R
            out = [];
            for i=1:length(t)
                ret=check_velocity_membership(t(i)); %output fuzzification
                ret = min(v_TFLC_R_th, ret);
                %Max value / real value of the inference
                out = [out max(ret)]; %MAX
            end
            centeroid_TFLC_R = sum(out.*t)/sum(out);
            %actuate the wheel
            centeroid_TFLC_L=centeroid_TFLC_L/2;
            centeroid_TFLC_R=centeroid_TFLC_R/2;
            %somehow I need to flip R and L to make it work, if not it'd be
            %possitive feedback
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(1),centeroid_TFLC_R,vrep.simx_opmode_blocking);%left
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,w_hdl(2),centeroid_TFLC_L,vrep.simx_opmode_blocking);%right
        end
        %-----Controller END here-----
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


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                      FUNCTIONS
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% =====================to check all value in distance fuzzification
function out = check_distance_membership(input)
    z = triangle (input, 0, 0, 0.50, 1, 1, 0); %zero
    nz = triangle (input, 0, 0.50, 1.00, 0, 1, 0); %near zero
    n = triangle (input, 0.50, 1.00, 1.50, 0, 1, 0); %near
    m = triangle (input, 1.00, 1.50, 2.00, 0, 1, 0); %medium
    nf = triangle (input, 1.50, 2.00, 2.50, 0, 1, 0); %near far
    f = triangle (input, 2.00, 2.50, 3.00, 0, 1, 0); %far
    vf = triangle (input, 2.50, 3.00, 3.00, 0, 1, 1); %very far
    out = [z;nz;n;m;nf;f;vf];
end
%% =====================to check all value in angle error fuzzification
function out = check_err_angle_membership(input)
    n = triangle (input, -175, -175, -88, 1, 1, 0); %negative
    sn = triangle (input, -175, -88, -43, 0, 1, 0); %small neg
    nnz = triangle (input, -88, -43, 0, 0, 1, 0); % near neg zero
    z = triangle (input, -43, 0, 43, 0, 1, 0); %zero
    npz = triangle (input, 0, 43, 88, 0, 1, 0); %near pos zero
    sp = triangle (input, 43, 88, 175, 0, 1, 0); %small pos
    p = triangle (input, 88, 175, 175, 0, 1, 1); %positive
    out = [n; sn; nnz; z; npz; sp; p];
end
%% =====================to check all value in velocity of motors in TFLC fuzzification
function out = check_velocity_membership(input)
    z = triangle (input, 0, 0, 2, 1, 1, 0);%zero
    s = triangle (input, 0, 2, 4, 0, 1, 0);%slow
    nm = triangle (input, 2, 4, 6, 0, 1, 0);%near medium
    m = triangle (input, 4, 6, 8, 0, 1, 0);%medium
    nh = triangle (input, 6, 8, 10, 0, 1, 0);%near high
    h = triangle (input, 8, 10, 12, 0, 1, 0);%high
    vh = triangle (input, 10, 12, 12, 0, 1, 1);%very high
    out = [z;s;nm;m;nh;h;vh];
end
%% =====================to check all value from sensor
function out = check_obstacle_membership(input)
    n = trapezoid (input, 0, 10, 30, 40, 0, 1, 1, 0);%near
    m = trapezoid (input, 30, 40, 60, 70, 0, 1, 1, 0);%medium
    f = trapezoid (input, 60, 70, 90, 100, 0, 1, 1, 1);%far
    out = [n;m;f];
end
%% =====================to check all value in LV and RV of OAFLC
function out = check_vl_vr_oaflc_membership(input)
    nh = triangle (input, -10, -5, 0, 0, 1, 0);%negative high
    n = triangle (input, -5, -2.5, 0, 0, 1, 0);%negative
    p = triangle (input, 0, 2.5, 5, 0, 1, 0);%positive
    hp = triangle (input, 0, 5, 10, 0, 1, 0);%high positive
    vhp = triangle (input, 5, 7.5, 10, 0, 1, 0);%very high positive
    out = [nh;n;p;hp;vhp];
end
%% Relate a fuzzification to another with the result of matrix & vec
% the relation is using minimum, because the relation in fuzzy rules are
% using AND
function [mat_rules_val,v_out_th] = relate_TFLC (set1, set2, rules_index)
    % set1 is rows, and set2 is columns
    % find correspoding values for each rules
    m = zeros(length(set1), length(set2));
    for i=1:length(set1)
        for j=1:length(set2)
            m(i,j)=min(set1(i),set2(j)); %MIN
        end
    end
    
    % find the output index of each rules values
    % then create vector, of possible maximum mambership value from output
    % membership class
    v_out = zeros(max(max(rules_index)),1);
    [h,w]=size(rules_index);
    for i = 1:h
        for j = 1:w
            v_out( rules_index(i,j) ) = max(v_out( rules_index(i,j) ), m (i,j)); %MAX
        end
    end
    mat_rules_val = m;
    v_out_th = v_out;
end
%% Relate a fuzzification to another with the result of rules table & vec
% the relation is using minimum, because the relation in fuzzy rules are
% using AND
function [mat_rules_val,v_out_th1, v_out_th2] = relate_OAFLC (set1, set2, set3, rules_index)
    % find correspoding values for each rules
    m = zeros(size(rules_index));
    c = 0;
    for i=1:length(set1)
        for j=1:length(set2)
            for k=1:length(set3)
                c = c+1;
                MIN = min(min(set1(i),set2(j)),set3(k));
                m(c,1)=MIN; %MIN output1
                m(c,2)=MIN; %MIN output2 
            end
        end
    end
    
    % find the output index of each rules values
    % then create vector, of possible maximum mambership value from output
    % membership class
    v_out1 = zeros(max(rules_index(:,1)),1); % vector output1
    v_out2 = zeros(max(rules_index(:,2)),1); % vector output2
    [h,w]=size(rules_index);
    for i = 1:h
        v_out1( rules_index(i,1) ) = max(v_out1( rules_index(i,1) ), m(i,1)); %MAX
        v_out2( rules_index(i,2) ) = max(v_out2( rules_index(i,2) ), m(i,2)); %MAX
    end
    mat_rules_val = m;
    v_out_th1 = v_out1;
    v_out_th2 = v_out2;
end
%% ===============triangle shaped fuzzy membership
function out = triangle (input, start, middle, stop, str_val, mid_val, sto_val)
    if input<start
        out = str_val;
        
    elseif (input>=start) && (input<middle)
        v = (mid_val-str_val)/(middle-start);
        out = str_val + v*(input-start);
    
    elseif (input==middle)
        out = mid_val;
        
    elseif (input>middle) && (input<=stop)
        v = (sto_val-mid_val)/(stop-middle);
        out = mid_val + v*(input-middle);
        
    elseif input>stop
        out = sto_val;
    end
end
%% ===============trapezoid shaped fuzzy membership
function out = trapezoid (input, start, middle1, middle2, stop, str_val, mid1_val, mid2_val, sto_val)
    if input<start
        out = str_val;
        
    elseif (input>=start) && (input<middle1)
        v = (mid1_val-str_val)/(middle1-start);
        out = str_val + v*(input-start);
    
    elseif (input==middle1)
        out = mid1_val;
        
    elseif (input>middle1) && (input<middle2)
        v = (mid2_val-mid1_val)/(middle2-middle1);
        out = mid1_val + v*(input-middle1);
        
    elseif (input==middle2)
        out = mid2_val;
        
    elseif (input>middle2) && (input<=stop)
        v = (sto_val-mid2_val)/(stop-middle2);
        out = mid2_val + v*(input-middle2);
        
    elseif input>stop
        out = sto_val;
    end
end