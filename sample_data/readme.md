ハミングカラーコードを用いたプロジェクターのキャリブレーションとグレーコードを用いた3次元復元のサンプルデータ

# フォルダ概要

* cap_chessboard  
ハミングカラーコードを用いたプロジェクター・カメラのキャリブレーション用画像セット

* cap_target  
グレーコード画像。プロジェクター・カメラの内部・外部パラメーターは上記のcap_chessboardの画像と同じ。


# 実行例

1. キャリブレーション画像のデコード  
`../hamming_color_code/hamming_decode.py -in_dir cap_chessboard`

2. キャリブレーション  
`./projcalib_from_decode_imgs.exe -xlist=decoded_x.txt -ylist=decoded_y.txt -imglist=cap_chess_imgs.txt -w=8 -h=5 -calib_mode=procam`

3. 3次元復元したい被写体のグレーコード画像のでコード  
`target_graycode_imgs.txt 512 384 -white_thresh=3 -black_thresh=50`

4. 3次元復元  
`-xMap=x.exr -yMap=y.exr -mask=x.png -cam=camera_data.yml -proj=proj_data.yml -rt=extrinsics.yml -texture=cap_target/DSC04088.JPG`
