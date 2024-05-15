function [jointVariables] = obtain_IK(ee_pos)
    x = ee_pos(1);
    y = ee_pos(2);
    z = ee_pos(3);
    phi = ee_pos(4);

%     j = zeros(4,4);
    l_1 = 0;
    l_2 = 109.6;
    l_3 = 109.6;
    l_4 = 56.46; %in mm

    s = z - l_1;
    r = sqrt(x*x+y*y);
    position = [x-l_4*cos(phi),y-l_4*sin(phi)];
    
    x1 = position(1);
    y1 = position(2);
    alpha = acos((l_2^2 +x1^2 + y1^2 - l_3^2)/(2*l_2*sqrt(x1^2+y1^2)));
    beta = acos((l_2^2 + l_3^2-x^1-y1^2)/(2*l_2*l_3));
    gamma = atan2(y1,x1);
    
    r;
    %Case 1 Elbow down
    theta1_1 = atan2(y,x);
    theta2_1 = gamma - alpha;
    theta3_1 = pi - beta;
    theta4_1 = phi - theta3_1 - theta2_1;
    elbow_down = dh2servo([theta1_1 theta2_1 theta3_1 theta4_1]);
    ed_degrees = elbow_down * 180 / pi

    % Elbow up
    theta1_2 = atan2(y,x);
    theta2_2 = gamma + alpha;
    theta3_2 = pi + beta;
    theta4_2 = phi - theta3_2 - theta2_2;
    elbow_up = dh2servo([theta1_2 theta2_2 theta3_2 theta4_2]);
    eu_degrees = elbow_up * 180 / pi

    %Elbow down reverse
    theta1_3 = theta1_1 + pi;
    theta2_3 = pi - (gamma - alpha);
    theta3_3 = -1*(pi - beta);
    theta4_3 = phi - theta3_3 - theta2_3;
    elbow_down_reverse = dh2servo([theta1_3 theta2_3 theta3_3 theta4_3]);
    

    %Elbow up reverse
    theta1_4 = theta1_1 + pi;
    theta2_4 = pi - (gamma + alpha);
    theta3_4 = -1*(pi + beta);
    theta4_4 = phi - theta3_4 - theta2_4;
    elbow_up_reverse = dh2servo([theta1_4 theta2_4 theta3_4 theta4_4]);
    

    jointVariables = [elbow_up 0]; % Return elbow-up solution.
%     j = [f1;f2;f3;f4];
%     j = vpa(j,2)
end