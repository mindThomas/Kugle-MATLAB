t = ObjData.Time;
q = ObjData.Quaternion;

flipIdx = [1; (find(abs(diff(q(:,1))) > 0.5)+1); length(t)];
qfix = q;
setsign = 1;
for (i = 1:(length(flipIdx)-1))
    qfix(flipIdx(i):flipIdx(i+1),:) = setsign * q(flipIdx(i):flipIdx(i+1),:); 
    setsign = -setsign;
end
q = qfix;

eul = quat2eul(q, 'ZYX');
figure(1);
subplot(3,1,1);
plot(t, rad2deg(eul(:,3))); title('Roll'); ylabel('deg');
subplot(3,1,2);
plot(t, rad2deg(eul(:,2))); title('Pitch'); ylabel('deg');
subplot(3,1,3);
plot(t, rad2deg(eul(:,1))); title('Yaw'); ylabel('deg');

figure(2);
subplot(4,1,1);
plot(t, q(:,1));
subplot(4,1,2);
plot(t, q(:,2));
subplot(4,1,3);
plot(t, q(:,3));
subplot(4,1,4);
plot(t, q(:,4));

figure(3);
x = ObjData.XYZ(:,2)';
y = -ObjData.XYZ(:,1)';
z = ObjData.XYZ(:,3)';
surface([x;x],[y;y],[z;z],[t';t'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
axis equal;    