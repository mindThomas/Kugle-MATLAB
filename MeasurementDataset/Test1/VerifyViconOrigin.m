Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
          q(2) q(1)  -q(4) q(3);
          q(3) q(4)  q(1)  -q(2);
          q(4) -q(3) q(2)  q(1)];           
Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
             p(2) p(1) p(4) -p(3);
             p(3) -p(4) p(1) p(2);
             p(4) p(3) -p(2) p(1)];  

devec = [0,1,0,0;0,0,1,0;0,0,0,1]; % 'v' in notes
vec = [0,0,0;1,0,0;0,1,0;0,0,1]; % '^' in notes
I_conj = diag([1,-1,-1,-1]);

addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Model\generated');
Constants_Kugle;
load('OnlyTiltingOnGround\2019-04-09_14-32-58-299.mat');

%%
q = ObjData.Quaternion;
eul = quat2eul(q, 'ZYX');
roll = eul(:,3);
pitch = eul(:,2);
time = ObjData.Time;
position = ObjData.XYZ / 1000;

B_p_vicon = [-9.9; 0; 911.6] / 1000;
K_p_vicon = zeros(length(q), 3);
for (i = 1:length(q))
    K_p_vicon(i,:) = ( devec * Phi(q(i,:)') * Gamma(q(i,:)')' * vec * B_p_vicon )';
end    
Pos = position - K_p_vicon;

fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Angle + position comparison');
subplot(2,1,1); plot(time, rad2deg(roll), time, rad2deg(pitch)); legend('Roll', 'Pitch'); title('Inclination'); ylabel('Angle [deg]'); xlabel('Time [s]');
subplot(2,1,2); yyaxis left; plot(time, position(:,3)); title('Vicon rigid body origin Z position'); ylabel('Distance [m]'); xlabel('Time [s]');
yyaxis right; plot(time, Pos(:,3));
xlabel('Time [s]');