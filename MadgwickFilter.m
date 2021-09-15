function q = MadgwickFilter(acc,gyro,mag,Fs,Beta)
% MadgwickAHRSファイルが必要
% Beta:strong:0.4, medium:0.15, weak:0.05
    
    % magデータの判定
    if mag == 0
        algorithm = "IMU";
    else
        algorithm = "MARG";
    end

    % 加速度から初期姿勢を取得
    nf = length(acc(:,1));
    time_int = 1/Fs;
    tmp_tt = 0:time_int:(nf-1)*time_int;
    tt = tmp_tt';
    stab1 = 1;
    stab2 = 100;
    q = zeros(nf, 4);
    %beta = sqrt(3/4)*pi*(5.0/180.0);    %原論文のまま
    ahrs =  MadgwickAHRS('SamplePeriod', time_int, 'Beta', Beta);
    if algorithm == "MARG"
        for i = 1:nf
            % gyro[0 0 0], acc, mag
            ahrs.Update([0 0 0],...
                [mean(acc(stab1:stab2,1)) mean(acc(stab1:stab2,2)) mean(acc(stab1:stab2,3))],...
                [mean(mag(stab1:stab2,1)) mean(mag(stab1:stab2,2)) mean(mag(stab1:stab2,3))]);
        end
        for i = 1:nf
            ahrs.Update(gyro(i,:), acc(i,:), mag(i,:));	% gyroscope units must be radians
            q(i, :) = ahrs.Quaternion;
            %R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
        end
    elseif algorithm == "IMU"
        for i = 1:nf
            % gyro[0 0 0], acc
            ahrs.UpdateIMU([0 0 0], [mean(acc(stab1:stab2,1)) mean(acc(stab1:stab2,2)) mean(acc(stab1:stab2,3))]);
        end
        for i = 1:nf
            ahrs.UpdateIMU(gyro(i,:), acc(i,:));	% gyroscope units must be radians
            q(i, :) = ahrs.Quaternion;
            %R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
        end        
    end
 
return
