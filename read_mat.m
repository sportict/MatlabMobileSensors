clear all;
clc;

%% matファイルの読み込み
[file, path] = uigetfile('*.mat');
if file==0
    return
end
load([path file]);

%% データ処理
% Mファイルからデータの読み込み
fs = M.fs;
tt = M.tt;
acc = M.acc;
gyro = M.gyro;
mag = M.mag;
orien = M.orien;
quat = M.quat;
euler = M.euler;
acc_global = M.acc_global;

% 方向データから回転行列の作成
orien_rad = deg2rad(orien);
RotMat = euler2rotMat(orien_rad(:,3),orien_rad(:,2),orien_rad(:,1)*-1);

%% センサ座標系加速度とグローバル座標系加速度のプロット
h2 = figure;
plot(tt,acc);
title('Sensor Coordinate System Acceleration')
xlabel('Time[s]');
ylabel('Acceleration[m/s^2]');

h3 = figure;
plot(tt,acc_global);
title('Global Coordinate System Acceleration')
xlabel('Time[s]');
ylabel('Acceleration[m/s^2]');

%% 姿勢角の表示（ティーポット）
h4 = figure;
[verts, faces, cindex] = teapotGeometry;
p = patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cindex,'FaceColor','interp');
ax = gca;
view(ax, [5 10]);
axis(ax,'equal')
set(ax,'CameraViewAngleMode','manual');
axis off
euler_txt = uicontrol('string',round(orien(1,:)',1),'style','text','position',[30,50,100,50],'tag','euler_txt');
euler_txt2 = uicontrol('string',["yaw";"pitch";"roll"],'style','text','position',[10,50,30,50]);

for i = 1:length(acc(:,1))
    rot_verts = RotMat(:,:,i) * verts';
    rot_verts = rot_verts';
    %rot_verts = quaternRotate(verts,compact(quat(i)));
    set(p,'Vertices',rot_verts)
    set(euler_txt,'string',round(orien(i,:)',1));
    pause(0.01)
end

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