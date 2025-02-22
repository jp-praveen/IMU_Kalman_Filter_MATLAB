%==========================================================================
% Author: Praveen Jawaharlal Ayyanathan
% Kalman Filter for 9DOF IMU

% The MPU9250 Magnetometer axis is made to match with the accelerometer and
% Gyroscope axis. This is done by swapping x and y axis of the 
% magnetometer and by taking the negative of the magnetometer z-axis.

% Declination of the place of IMU calibration: 1.26 degree West. For
% Declination along west subtract it from the final Yaw and for a Declination
% along East add it with the Yaw. This is done to get the heading along the
% geographic north.
%==========================================================================
%%
clear
clc
close all

%%
% CONNECTING THE ARDUINO AND MATLAB. ON MY LAPTOP THE ARDUINO IS IN COM5.
UNO = arduino('COM5', 'Mega2560', 'Libraries', 'I2C');

% SAMPLE RATE IN Hz
FS = 100; 
dt = 1/FS;

IMU = mpu9250(UNO,'SampleRate',FS,'OutputFormat','matrix');

% DEFINING THE NECESSARY ARRAYS FOR CALIBRATION
Accelerometer_arr = [];
AccelerometerX = [];
AccelerometerY = [];
AccelerometerZ = [];
Magnetometer_arr = [];
Gyroscope_arr = [];

j = 0;

%% ACCELERATOMETER, GYROSCOPE AND MAGNETOMETER CALIBRATION
% To calibrate the accelerometer and gyroscope, just lay them on a flat
% surface for a few seconds. Ideally the accelerometer should read [0,0,-1g]
% and gyroscope should read [0,0,0]. But due to noises the accelerometer 
% and gyro will be having some other values. The average of this noise is 
% taken as the bias for the acc and gyro. The acc data is subtracted from 
% the acc bias while the gyro bias found is given as an input to the 
% Kalman Filter.

% The magnetometer is calibrated using the magcal() function of MATLAB.

%%

% ACCELEROMETER AND GYRO CALIBRATION
disp('Starting Acc and Gyro calib')
tic;
% MagnetometerX = [];
% MagnetometerY = [];
% MagnetometerZ = [];
while toc<25
    [Accelerometer,Gyroscope,Magnetometer] = readSensorDataMPU9250(IMU);
%     MagnetometerX = Magnetometer(:,1);
%     MagnetometerY = Magnetometer(:,2);
%     MagnetometerZ = Magnetometer(:,3);
    
    AccelerometerX = Accelerometer(:,1);
    AccelerometerY = Accelerometer(:,2);
    AccelerometerZ = Accelerometer(:,3);
    Accelerometer = [];
    Accelerometer = [AccelerometerX, AccelerometerY, AccelerometerZ];
    
    Accelerometer_arr = [Accelerometer_arr; Accelerometer];
    Gyroscope_arr = [Gyroscope_arr; Gyroscope];
end
disp('Ending Acc and Gyro calib')

% MAGNETOMETER CALIBRATION
% disp('Start Mag calib')
% while toc<20
%     j = j+1;
%     [Accelerometer,Gyroscope,Magnetometer] = readSensorDataMPU9250(IMU);
% %     Magnetometer_arr = [Magnetometer_arr; Magnetometer];
%     Mag_avg(j, :) = mean(Magnetometer);
% end
% disp('Stop Mag calib')
% pause(3)

% MAGNETOMETER CALIBRATION COEFFICIENTS
% [A_MAG_CAL, b, expMFS] = magcal(Mag_avg);

% PRE-DEFINED CALIBRATION COEFF'S FOR MAGNETOMETER TO SAVE TIME
%A_MAG_CAL = [0.939579873911505,0.0157380187228683,0.0267911217602576;0.0157380187228683,1.00533490810877,-0.0643697912001522;0.0267911217602576,-0.0643697912001522,1.06387946185326];
%b = [7.50205199436905,60.9380674475080,-57.3625520739329];
A_MAG_CAL = [0.914721236717404,0.006591763526623,0.011361282977070;0.006591763526623,1.070038649189700,-0.035519277139419;0.011361282977070,-0.035519277139419,1.023043668814983];
b = [14.745729878906296,53.848657204415600,-56.764578034864940];
% CALUCULATING THE ACCELEROMETER AND GYRO BIAS
BIAS_ACCX = mean(Accelerometer_arr(:,1));
BIAS_ACCY = mean(Accelerometer_arr(:,2));
BIAS_ACCZ = mean(9.81-Accelerometer_arr(:,3));

BIAS_GYROX = mean(Gyroscope_arr(:,1));
BIAS_GYROY = mean(Gyroscope_arr(:,2));
BIAS_GYROZ = mean(Gyroscope_arr(:,3));

%% CONSTANTS THAT WILL BE USED FOR THE EXECUTION OF KALMAN FILTER
% Accelerometer_old: Will be used as a previous data for the acclerometer
%                    low pass filter.
% DECLINATION: To be added with Yaw to get the heading along geographical
%              north.     
% LOW_PASS_WEIGHT: The coefficient/weight for accelerometer low pass filter.
% P: Initial Covariance matrix.
% C: Observation matrix.
% R: Measurement Noise Covariance.
% Q: Process Noise Covariance.
% X_PREV_BEST_ESTIMATE: Initial best estimate. Make sure to line up the
%                       magnetometer's x-axis along the north magnetic
%                       pole.
%%

DECLINATION = -1.26;
Accelerometer_old = [0 0 9.81];
LOW_PASS_WEIGHT = 0.75;

C = [1, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 1, 0];
 
P =[1e-6,0,0,0,0,0;
    0,1e-6,0,0,0,0;
    0,0,1e-6,0,0,0;
    0,0,0,1e-6,0,0;
    0,0,0,0,1e-6,0;
    0,0,0,0,0,1e-6]; 
   
R = [1 0 0;0 1 0; 0 0 1];  

PNC_CONST = 1e7;

Q = [0.1*PNC_CONST,0,0,0,0,0;
    0,0.1*PNC_CONST,0,0,0,0;
    0,0,0.1*PNC_CONST,0,0,0;
    0,0,0,0.1*PNC_CONST,0,0;
    0,0,0,0,0.1*PNC_CONST,0;
    0,0,0,0,0,0.1*PNC_CONST]; 

X_PREV_BEST_ESTIMATE = [0; BIAS_GYROX; 0; BIAS_GYROY; 0; BIAS_GYROZ];

sample = 1;
tic
IMU_OLD_TIME = toc;
figure(1)
while true
    figure(1)
    [Accelerometer,Gyroscope,Magnetometer] = read(IMU);
    
    % FINDING THE INTERGRATION TIME
    IMU_NEW_TIME = toc;
    dt = IMU_NEW_TIME - IMU_OLD_TIME;
    IMU_OLD_TIME = IMU_NEW_TIME;
    
    % REMOVING THE ACCELEROMETER'S INITIAL BIAS
    Accelerometer(:,1) = Accelerometer(:,1) - BIAS_ACCX;
    Accelerometer(:,2) = Accelerometer(:,2) - BIAS_ACCY;
    Accelerometer(:,3) = Accelerometer(:,3) - BIAS_ACCZ;
    
    % ACCELEROMETER'S LOW PASS FILTER EXECUTION
    Accelerometer = Accelerometer_old*LOW_PASS_WEIGHT + Accelerometer*(1-LOW_PASS_WEIGHT);
    
    A = [1, -dt, 0,  0, 0, 0;
         0,  1,  0,  0, 0, 0;
         0,  0,  1, -dt, 0, 0;
         0,  0,  0,  1, 0, 0;
         0,  0,  0,  0, 1, -dt;
         0,  0,  0,  0, 0, 1];

    B = [dt, 0, 0;
         0,  0, 0;
         0,  dt, 0;
         0,  0, 0;
         0, 0, dt;
         0,  0, 0]; 
    
    for i = 1:10
        
        % MAGNETOMETER CALIBRATION AND MATCHING THE MAGNETOMETER'S AXIS 
        % WITH ACCELEROMETER AND GYRO.
        Magnetometer(i,:) = (Magnetometer(i,:) - b)*A_MAG_CAL;
        Magnetometer(i,:) = Magnetometer(i,:)/norm(Magnetometer(i,:));
        
        MagnetometerX = Magnetometer(i,2);
        MagnetometerY = Magnetometer(i,1);
        MagnetometerZ = -Magnetometer(i,3);
        Magnetometer(i,:) = [MagnetometerX, MagnetometerY, MagnetometerZ];
        
        % DYNAMIC MODEL FOR THE EULER ANGLES. ANGLES ARE IN RADIANS.
        PHI_ACCEL_T_RAD(i) = atan2(Accelerometer(i,2),sqrt(Accelerometer(i,1)^2+Accelerometer(i,3)^2));
        THETA_ACCEL_T_RAD(i) = atan2(-Accelerometer(i,1),sqrt(Accelerometer(i,2)^2 + Accelerometer(i,3)^2));

        % SWAPPED EXPRESSION FOR PHI AND THETA.
%         MX = Magnetometer(i,1)*cos(THETA_ACCEL_T_RAD(i)) + Magnetometer(i,3)*sin(THETA_ACCEL_T_RAD(i));
%         MY = Magnetometer(i,1)*sin(PHI_ACCEL_T_RAD(i))*sin(THETA_ACCEL_T_RAD(i)) + Magnetometer(i,2)*cos(PHI_ACCEL_T_RAD(i)) - Magnetometer(i,3)*sin(PHI_ACCEL_T_RAD(i))*cos(THETA_ACCEL_T_RAD(i));

        % PSI
        MY = -Magnetometer(i,2)*cos(PHI_ACCEL_T_RAD(i))+Magnetometer(i,3)*sin(PHI_ACCEL_T_RAD(i));
        MX = Magnetometer(i,1)*cos(THETA_ACCEL_T_RAD(i)) + Magnetometer(i,2)*sin(THETA_ACCEL_T_RAD(i))*sin(PHI_ACCEL_T_RAD(i)) + Magnetometer(i,3)*sin(THETA_ACCEL_T_RAD(i))*cos(PHI_ACCEL_T_RAD(i)); 
       
        PSI_MAG(i) = atan2(MY,MX); 
        
        % EULER ANGLES FOUND USING THE DYNAMICS.
        Z_T = [PHI_ACCEL_T_RAD(i); THETA_ACCEL_T_RAD(i);PSI_MAG(i)];
        
        % EULER ANGLE RATE OF CHANGE FROM GYRO DATA.
        DOT_MATRIX = [1, sin(PHI_ACCEL_T_RAD(i))*tan(THETA_ACCEL_T_RAD(i)), cos(PHI_ACCEL_T_RAD(i))*tan(THETA_ACCEL_T_RAD(i));
                      0, cos(PHI_ACCEL_T_RAD(i)),                           -sin(PHI_ACCEL_T_RAD(i));
                      0, sin(PHI_ACCEL_T_RAD(i))*sec(THETA_ACCEL_T_RAD(i)), cos(PHI_ACCEL_T_RAD(i))*sec(THETA_ACCEL_T_RAD(i))];

        EULER_DOT = DOT_MATRIX*Gyroscope(i,:).';

        U_T = [EULER_DOT(1); EULER_DOT(2); EULER_DOT(3)];

        % KALMAN PREDICTION EQUATIONS 
        X_BEST_ESTIMATE(:,i) = A*X_PREV_BEST_ESTIMATE + B*U_T;
        P = A*P*A.' + Q;

        % KALMAN UPDATE EQUATIONS
        Y_T = Z_T - C*X_BEST_ESTIMATE(:,i);
        S_T = C*P*C.' + R;
        K_T = P*C.'*S_T^(-1);
        X_BEST_ESTIMATE(:,i) = X_BEST_ESTIMATE(:,i) + K_T*Y_T;
        P = (eye(6,6) - K_T*C)*P;
         
        X_PREV_BEST_ESTIMATE = X_BEST_ESTIMATE(:,i);
    end
    Accelerometer_old = mean(Accelerometer);
    sample = sample + 1;
    
    ROLL  = mean(X_BEST_ESTIMATE(1,:))*180/pi;
    PITCH = mean(X_BEST_ESTIMATE(3,:))*180/pi;
    YAW   = -(mean(X_BEST_ESTIMATE(5,:))*180/pi + DECLINATION);
    hold on 
        
    % PLOTTING THE EULER ANGLES IN DEGREES
    plot(sample, ROLL, '-or');
    plot(sample, PITCH, '-og');
    plot(sample, YAW, '-ob');
end
     
    
    
              
    