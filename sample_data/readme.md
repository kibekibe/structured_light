�n�~���O�J���[�R�[�h��p�����v���W�F�N�^�[�̃L�����u���[�V�����ƃO���[�R�[�h��p����3���������̃T���v���f�[�^

# �t�H���_�T�v

* cap_chessboard  
�n�~���O�J���[�R�[�h��p�����v���W�F�N�^�[�E�J�����̃L�����u���[�V�����p�摜�Z�b�g

* cap_target  
�O���[�R�[�h�摜�B�v���W�F�N�^�[�E�J�����̓����E�O���p�����[�^�[�͏�L��cap_chessboard�̉摜�Ɠ����B


# ���s��

1. �L�����u���[�V�����摜�̃f�R�[�h  
`../hamming_color_code/hamming_decode.py -in_dir cap_chessboard`

2. �L�����u���[�V����  
`./projcalib_from_decode_imgs.exe -xlist=decoded_x.txt -ylist=decoded_y.txt -imglist=cap_chess_imgs.txt -w=8 -h=5 -calib_mode=procam`

3. 3����������������ʑ̂̃O���[�R�[�h�摜�̂ŃR�[�h  
`target_graycode_imgs.txt 512 384 -white_thresh=3 -black_thresh=50`

4. 3��������  
`-xMap=x.exr -yMap=y.exr -mask=x.png -cam=camera_data.yml -proj=proj_data.yml -rt=extrinsics.yml -texture=cap_target/DSC04088.JPG`
