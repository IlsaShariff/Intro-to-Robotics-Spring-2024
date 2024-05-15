function [ servoJointAngles , errorCode ] = dh2servo ( jointAngles )
    % Apply offsets determined in 5.6
    servoJointAngles = jointAngles;
%     angles_degrees = jointAngles*180/pi
    servoJointAngles (2) = servoJointAngles (2) - pi /2;
    % map angles to [-pi , pi ].
    nAngles = length ( servoJointAngles );
    for i = 1: nAngles
        theta_i = real(servoJointAngles(i));
        % Wrap angle value if it is greater than 180 degrees .
        if abs ( theta_i ) > pi
%             theta_i;
            theta_i = mod(theta_i + pi, 2*pi) - pi;
        end
        % constrain angles between [ -150 , 150] degrees .
        if abs ( theta_i ) > deg2rad (150)
            errorCode = i; % Error code corresponds to joint id.
            servoJointAngles = [];
            return ;
        end
    servoJointAngles (i) = theta_i ;
    end
end