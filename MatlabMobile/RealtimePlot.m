clearvars -except m
clc;

%% デバイスの設定
if exist('m','var')
    discardlogs(m)
else
    m = mobiledev;
end

%% 初期設定
% 加速度(Acceleration):[m/s^2]
% 角速度(AngularVelocity):[rad/s]
% 磁場(Magnetic):[μT]
% 方向(Orientation):[方位、傾斜、回転　deg]

m.SampleRate = 'high';
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
m.OrientationSensorEnabled = 1;
m.MagneticSensorEnabled = 1;
m.PositionSensorEnabled = 0;

fs = m.SampleRate;
i = 1;

disp('スマートフォンの開始ボタンを押して下さい')

while isempty(m.Acceleration); end
m.Logging = 1;
h1 = figure;
subplot(1,2,2)
[verts,p] = CreatePot();
euler_txt = uicontrol('string',[0,0,0],'style','text','position',[0,0,50,15]);
ax = gca;

while (1==1)
    ac = m.Acceleration;
    gy = m.AngularVelocity;
    ma = m.MagneticField;
    ori = m.Orientation;
    ori = deg2rad(ori);
    
    if isempty(ac); break; end
    
    Rot = euler2rotMat(ori(3),ori(2),ori(1)*-1);
    rot_verts = Rot * verts';
    
    set(p,'Vertices',rot_verts');
    set(euler_txt,'string',ori);
    
    accel(i,:) = ac;
    if i == 1
        t(1) = 0;
    else
        t(i) = t(i-1) + 1/fs;
    end
    subplot(1,2,1)
    plot(t,accel);
    drawnow
    i = i+1;
end
m.Logging = 0;

%% スマートフォンからデータログを抽出
[acc, ta] = accellog(m);
[gyro, tg] = angvellog(m);
[mag, tm] = magfieldlog(m);
[orien, to] = orientlog(m);

%% データ処理
% Madgwick AHRSアルゴリズム
q = MadgwickFilter(acc,gyro,fs,0.4);
quat = quaternion(q);
euler = eulerd(quat,'ZXY','frame');
    
% センサ座標系をグローバル座標系に変換(クォータニオンを利用)
acc_global = quaternRotate(acc,compact(quat));

h2 = figure;
plot(ta,acc);
title('Sensor Acceleration [m/s^2]')

h3 = figure;
plot(ta,acc_global);
title('Global Acceleration [m/s^2]')

%%
h4 = figure;
[verts, faces, cindex] = teapotGeometry;
p = patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cindex,'FaceColor','interp');
ax = gca;
view(ax, [5 10]);
axis(ax,'equal')
set(ax,'CameraViewAngleMode','manual');
rotate3d off;
axis off
euler_txt = uicontrol('string',euler,'style','text','position',[0,0,50,15],'tag','euler_txt');

for i = 1:length(acc(:,1))
    rot_verts = quaternRotate(verts,compact(quat(i)));
    set(p,'Vertices',rot_verts)
    set(euler_txt,'string',euler(i,:));
    pause(0.01)
end

%% データの保存
M.fs = fs;
M.tt = ta;
M.acc = acc;
M.gyro = gyro;
M.mag = mag;
M.orien = orien;
M.quat = quat;
M.acc_global = acc_global;

save('mobile2.mat','M')

%--------------------------------------------------------------------------
function [verts,p] = CreatePot()

[verts, faces, cindex] = teapotGeometry;
p = patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cindex,'FaceColor','interp');
ax = gca;
view(ax, [5 10]);
axis(ax,'equal')
set(ax,'CameraViewAngleMode','manual');
rotate3d off;
axis off

end