import numpy as np
import argparse
import cv2



H4 = '6767645454676232326754576757646751323132645754645131315451546231576267323767375757313751573732626464626237315151373764'

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
        print 'Not implemented.'
        return
    
    pattern_length = len(str_array)
    color_array = np.zeros((pattern_length,3), dtype=np.uint8)
    for i, character in enumerate(str_array):
        num = int(character) - 1
        color_array[i] = color[num]
        
    return color_array
    
def gen_vertical_stripe_image(color_array, width, height):
    line = np.zeros((1, color_array.shape[0], 3), np.uint8)
    line[:] = color_array.reshape((1,color_array.shape[0], 3))[:]
    line = cv2.resize(line, (width, 1), interpolation=cv2.INTER_NEAREST)
    dst = cv2.resize(line, (width, height))
    return dst
    

def gen_horizontal_stripe_image(color_array, width, height):
    line = np.zeros((color_array.shape[0], 1, 3), np.uint8)
    line[:] = color_array.reshape((color_array.shape[0], 1, 3))[:]
    line = cv2.resize(line, (1, height), interpolation=cv2.INTER_NEAREST)
    dst = cv2.resize(line, (width, height))
    return dst


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--image_width',  '-img_w', type=int, default=512)
    parser.add_argument('--image_height', '-img_h', type=int, default=384)
    parser.add_argument('--dst_x',        '-dst_x', type=str, default='pattern_x.png')
    parser.add_argument('--dst_y',        '-dst_y', type=str, default='pattern_y.png')
            
    args = parser.parse_args()
    
    width  = args.image_width
    height = args.image_height
    
    dst_x = args.dst_x
    dst_y = args.dst_y
    
    color_array = gen_color_from_string(H4, 4)
    
    pattern_img_x = gen_vertical_stripe_image(color_array, width, height)
    pattern_img_y = gen_horizontal_stripe_image(color_array, width, height)
    
    
    cv2.imwrite(dst_x, pattern_img_x)
    cv2.imwrite(dst_y, pattern_img_y)
    
    

        
    