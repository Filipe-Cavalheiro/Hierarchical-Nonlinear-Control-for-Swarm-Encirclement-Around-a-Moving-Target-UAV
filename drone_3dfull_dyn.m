function [dp,dv,dR,dom] = drone_3dfull_dyn(v,R,om,T,tau,P)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

    % auxiliary variables
    zW = [0;0;1];
    zB = R(:,3);
    
    % equations of motion
    dp = v;
    dv = -P.g*zW + T/P.m*zB - R*P.D*R'*v;
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);

%     if t > 0.001
%         test = 1;
%     end
    
end

