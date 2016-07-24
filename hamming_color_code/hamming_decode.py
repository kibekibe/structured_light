
import numpy as np
import os
import sys
import argparse
import cv2


LEFT = 1
TOP  = 2
DIAG = 0

H4 = '6767645454676232326754576757646751323132645754645131315451546231576267323767375757313751573732626464626237315151373764'




# ビット情報を色にエンコードする。
def gen_color():
    color = np.zeros((7,3), dtype=np.uint8)
    for i in range(1,8):
        r = i & 1
        g = (i>>1) & 1
        b = (i>>2) & 1
        color[i-1] = b,g,r
    color *= 255
    return color

def gen_color_from_string ( str_array, k ):
    color = gen_color()
    if not k == 4:
        return
    
    pattern_length = len(str_array)
    color_array = np.zeros((pattern_length,3), dtype=np.uint8)
    for i, character in enumerate(str_array):
        num = int(character) - 1
        color_array[i] = color[num]
        
    return color_array
    
# 
# get_segment_imageから呼ばれる
def get_seg(val, color_array):
    dif = color_array - val / val.max() * 255.0
    dif = abs(dif)
    dif = dif.sum(axis=1) # 0:行数が1行になる。, 1:列数が1になる
    id = np.argmin(dif)
    return id
        
# 画像imgの輝度値をエンコードした色に対応させる。その結果を出力する。
def get_segment_image(img, mask, id_color):
    h, w = img.shape[:2]

    seg = np.zeros(img.shape[:2], dtype=np.int)
    for j in xrange(h):
        for i in xrange(w):
            if mask[j,i] > 0:
                seg[j,i] = get_seg(img[j,i], id_color) + 1 # +1はH4がゼロ無いから
    
    return seg

def get_seg_seq(seg_img_scanline):
    seg_seq = []
    start_pixel_id_list = []
    end_pixel_id_list = []
    
    w = len(seg_img_scanline)
    px = 0
    cur_seg = seg_img_scanline[px]
    
    #is_searching_start = True
    is_searching_end   = False
    
    if cur_seg > 0:
        seg_seq.append(cur_seg)
        start_pixel_id_list.append(px)
        #is_searching_start = False
        is_searching_end   = True
        
    for i in xrange(w):
        
        if not seg_img_scanline[i] == cur_seg:
            if is_searching_end:
                end_pixel_id_list.append(i)
                #is_searching_start = True
                is_searching_end   = False
                
            if not seg_img_scanline[i] == 0:
                start_pixel_id_list.append(i)
                seg_seq.append(seg_img_scanline[i])
                #is_searching_start = False
                is_searching_end   = True
                
            cur_seg = seg_img_scanline[i]
            
    # 最後のセグメントidから画像端まで来た場合は、端をend_pixel_id_listに追加する
    if len(start_pixel_id_list) == len(end_pixel_id_list) + 1:
        end_pixel_id_list.append(w)
    
    return seg_seq, start_pixel_id_list, end_pixel_id_list
    
def get_seg_str_from_seg_id(seg_seq):
    corres = []
    for i in xrange(len(seg_seq)-3):
        pattern = '%d%d%d%d'%(seg_seq[i], seg_seq[i+1], seg_seq[i+2], seg_seq[i+3])
        find = H4.find(pattern)# ミスったら-1を返しているはず。
        corres.append(find)   
    return corres
    
# id_color : IDを色にエンコードしたもの
# color_array : ハミングコード。IDの配列
def decode_x(cap_v, mask, color_array, id_color):
    seg_img = get_segment_image(cap_v, mask, id_color)
    h, w = mask.shape[:2]

    decoded_img = np.zeros(mask.shape[:2], np.float32)
    
    for j in range(h):            
        seg_img_scanline = seg_img[j]
        
        if seg_img_scanline.max() == 0:
            continue
        
        seg_seq, start_pixel_id_list, end_pixel_id_list = get_seg_seq(seg_img_scanline)
        corres = get_seg_str_from_seg_id(seg_seq)
        
        for id in xrange(len(start_pixel_id_list)-4):
            decoded_img[j, start_pixel_id_list[id]:end_pixel_id_list[id]] = corres[id]

    return decoded_img

def decode_y(cap_h, mask, color_array, id_color):
    transposed_h = np.transpose(cap_h, (1,0,2))
    transposed_m = np.transpose(mask,(1,0))
    decoded_v = decode_x(transposed_h, transposed_m, color_array, id_color)
    decoded_h = np.transpose(decoded_v, (1,0))
    
    return decoded_h
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
       
    parser.add_argument('--input_dir', '-in_dir', type=str)
    parser.add_argument('--lit_thr', '-lit_thr', type=int, default=40)
    parser.add_argument('--output_dir', '-out_dir', type=str, default='decoded')
    parser.add_argument('--output_8bit_dir', '-out_8bit_dir', type=str, default='decoded_8bit')
    parser.add_argument('--output_mask_dir', '-out_mask_dir', type=str, default='mask')
    
    parser.add_argument('--blur_kernel', '-k', type=int, default=7)
            
    args = parser.parse_args()
        
    color_array = gen_color_from_string(H4, 4).astype(np.float32) # ハミングコード
    id_color    = gen_color() # ビットを色情報にエンコードしたもの
        

    # 指定フォルダ内の全画像を対象にデコード処理
    # 

    input_dir       = args.input_dir
    output_dir      = args.output_dir
    output_8bit_dir = args.output_8bit_dir
    output_mask_dir = args.output_mask_dir
    
    if not os.path.exists(input_dir):
        print '{} does not exist.'.format(input_dir)
        sys.exit()
        
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
        
    if not os.path.exists(output_8bit_dir):
        os.mkdir(output_8bit_dir)
        
    if not os.path.exists(output_mask_dir):
        os.mkdir(output_mask_dir)
        
    files = os.listdir(input_dir)
    
    if not len(files) % 3 == 0:
        print 'Number of images must be in multiples of 3.'
        sys.exit()
        
    white_files = files[::3]
    y_files     = files[1::3]
    x_files     = files[2::3]
    
    set_num = len(x_files)
    
    for w_file, x_file, y_file in zip(white_files, x_files, y_files):
        print w_file, x_file, y_file
        
        cap_x = cv2.imread(os.path.join(input_dir, x_file)).astype(np.float32)
        cap_y = cv2.imread(os.path.join(input_dir, y_file)).astype(np.float32)
        
        mask  = cv2.imread(os.path.join(input_dir, w_file), 0)
        mask[mask<=args.lit_thr] = 0
        mask[mask>0]             = 255
        
        decoded_x = decode_x(cap_x, mask, color_array, id_color)
        decoded_y = decode_y(cap_y, mask, color_array, id_color)
        
        k = args.blur_kernel
        sigma = (k/2.0-1)*0.3 + 0.8
        if k >= 3:
            decoded_x = cv2.GaussianBlur(decoded_x, (k, k), sigma)
            decoded_y = cv2.GaussianBlur(decoded_y, (k, k), sigma)
        
        output_x_file = os.path.join(output_dir, os.path.splitext(w_file)[0] + '_x.exr')
        output_y_file = os.path.join(output_dir, os.path.splitext(w_file)[0] + '_y.exr')
        cv2.imwrite(output_x_file, decoded_x)
        cv2.imwrite(output_y_file, decoded_y)
        
        output_8bit_x_file = os.path.join(output_8bit_dir, os.path.splitext(w_file)[0] + '_x.png')
        output_8bit_y_file = os.path.join(output_8bit_dir, os.path.splitext(w_file)[0] + '_y.png')
        cv2.imwrite(output_8bit_x_file, decoded_x.astype(np.uint8))
        cv2.imwrite(output_8bit_y_file, decoded_y.astype(np.uint8))
        
        output_mask_file = os.path.join(output_mask_dir, os.path.splitext(w_file)[0] + '.png')
        cv2.imwrite(output_mask_file, mask)
    
        
    