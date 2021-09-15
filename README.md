# MatlabMobileSensors
プログラムの実行の際に、スマートフォンアプリの「MATLAB Mobile」が必要です。
また、MathWorksアカウントに接続しておく必要があります。

「RealtimePlot.m」ファイルを開き、実行します。
「スマートフォンの開始ボタンを押して下さい」の画面がMATLAB上に表示された後に、スマートフォンの「開始」を押します。

https://user-images.githubusercontent.com/62732939/133471896-bd612d69-9c8e-49a1-8be2-409194fd2ab8.mp4

上記の画面が表示され、スマートフォンに内蔵されたセンサの加速度を取得し、プロットします。
スマートフォンの方向を取得し、ティーポットをスマートフォンの姿勢に回転させます。

終了する場合は、スマートフォンの「停止」を押します。

停止後、しばらくするとセンサ座標系の加速度とグローバル座標系の加速度のグラフと、ティーポットの姿勢がアニメーションで表示されます。

<img width="421" alt="SCS" src="https://user-images.githubusercontent.com/62732939/133474615-be7fa492-547e-483f-abcc-0f3613b06a7e.png">
<img width="421" alt="GCS" src="https://user-images.githubusercontent.com/62732939/133474634-09c25471-abcd-4f83-83f0-2b0fb976ee07.png">

https://user-images.githubusercontent.com/62732939/133473094-bad51f46-1f87-4c68-9e9a-a5b53136b3fc.mp4

測定したデータは、名前をつけてmatファイルとして保存ができます。
<img width="530" alt="savemat" src="https://user-images.githubusercontent.com/62732939/133471272-310151d8-fc98-4cf3-b08f-c6b58417bc97.png">


保存したmatファイルは「read_mat.m」を実行することで読み込むことができ、加速度のグラフとティーポットの姿勢のアニメーションを表示できます。




