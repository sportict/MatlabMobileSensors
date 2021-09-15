clearvars -except m
close all;
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
euler_txt = uicontrol('string',[0;0;0],'style','text','position',[350,50,100,50]);
euler_txt2 = uicontrol('string',["yaw";"pitch";"roll"],'style','text','position',[330,50,30,50]);
ax = gca;

while (1==1)
    ac = m.Acceleration;
    gy = m.AngularVelocity;
    ma = m.MagneticField;
    ori = m.Orientation;
    ori_rad = deg2rad(ori);
    
    if isempty(ori); break; end
    
    Rot = euler2rotMat(ori_rad(3),ori_rad(2),ori_rad(1)*-1);
    rot_verts = Rot * verts';
    rot_verts = rot_verts';
    
    set(p,'Vertices',rot_verts);
    set(euler_txt,'string',round(ori',1));
    
    accel(i,:) = ac;
    orie(i,:) = ori;
    if i == 1
        t(1) = 0;
    else
        t(i) = t(i-1) + 1/fs;
    end
    subplot(1,2,1)
    plot(t,accel);
    xlabel('Time[s]');
    ylabel('Acceleration[m/s^2]');
    ax1 = gca;
    ax1.Toolbar.Visible = 'off';
    if i < 200
        xlim([0 2]);
    else
        xlim([i/fs-2 i/fs]);
    end
    drawnow
    i = i + 1;
end
m.Logging = 0;

%% スマートフォンからデータログを抽出
[acc, ta] = accellog(m);
[gyro, tg] = angvellog(m);
[mag, tm] = magfieldlog(m);
[orien, to] = orientlog(m);

% データ数を一致させる
[acc,gyro,mag,orien] = MatchLength(acc,gyro,mag,orien);

%% データ処理
% Acc,Gyroからクォータニオンを算出
qIMU = MadgwickFilter(acc,gyro,0,fs,0.4);
quatIMU = quaternion(qIMU);
eulerIMU = eulerd(quatIMU,'ZXY','frame');
acc_global_IMU = quaternRotate(acc, compact(quatIMU));

% Acc,Gyro,Magからクォータニオンを算出
qMARG = MadgwickFilter(acc,gyro,0,fs,0.4);
quatMARG = quaternion(qMARG);
eulerMARG = eulerd(quatMARG,'ZXY','frame');
acc_global_MARG = quaternRotate(acc, compact(quatMARG));

% 方向データから回転行列の作成
orien_rad = deg2rad(orien);
RotMat = euler2rotMat(orien_rad(:,3),orien_rad(:,2),orien_rad(:,1)*-1);

%% センサ座標系加速度とグローバル座標系加速度のプロット
h2 = figure;
plot(ta,acc);
title('Sensor Coordinate System Acceleration')
xlabel('Time[s]');
ylabel('Acceleration[m/s^2]');

h3 = figure;
plot(ta,acc_global_IMU);
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

%% データの保存
M.fs = fs;
M.tt = ta;
M.acc = acc;
M.gyro = gyro;
M.mag = mag;
M.orien = orien;
M.quat = quatIMU;
M.euler = eulerIMU;
M.acc_global = acc_global_IMU;

[file, path] = uiputfile('*.mat');
if file==0
    return
end
save([path file],'M')

%--------------------------------------------------------------------------
function [verts,p] = CreatePot()

    [verts, faces, cindex] = teapotGeometry;
    p = patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cindex,'FaceColor','interp');
    ax = gca;
    view(ax, [5 10]);
    axis(ax,'equal')
    set(ax,'CameraViewAngleMode','manual');
    ax.Toolbar.Visible = 'off';
    rotate3d off;
    axis off

end

%--------------------------------------------------------------------------
function [acc,gyro,mag,orien] = MatchLength(ac,gy,ma,or)

    nfa = length(ac);
    nfg = length(gy);
    nfm = length(ma);
    nfo = length(or);
    mx = max([nfa,nfg,nfm,nfo]);
    acc = ac;
    gyro = gy;
    mag = ma;
    orien = or;
    if nfa ~= mx
        k = 1;
        diff = mx - nfa;
        for i = 1:diff
            acc(nfa+k,:) = ac(end,:);
            k = k + 1;
        end
    end
    if nfg ~= mx
        k = 1;
        diff = mx - nfg;
        for i = 1:diff
            gyro(nfg+k,:) = gyro(end,:);
            k = k + 1;
        end
    end
    if nfm ~= mx
        k = 1;
        diff = mx - nfm;
        for i = 1:diff
            mag(nfm+k,:) = ma(end,:);
            k = k + 1;
        end
    end
    if nfo ~= mx
        k = 1;
        diff = mx - nfo;
        for i = 1:diff
            orien(nfo+k,:) = or(end,:);
            k = k + 1;
        end
    end    

end