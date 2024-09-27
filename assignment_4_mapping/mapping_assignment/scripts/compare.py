from PIL import Image, ImageDraw
import argparse

def compare_images(image1_path, image2_path, output_path):
    img1 = Image.open(image1_path).convert("RGB")
    img2 = Image.open(image2_path).convert("RGB")
    
    if img1.size != img2.size:
        raise ValueError("Images must have the same dimensions.")
    
    border_width = 2  
    img_size = (img1.width, img1.height)
    
    diff_img = Image.new('RGB', img_size, "white")
    draw = ImageDraw.Draw(diff_img)

    pixels1 = img1.load()
    pixels2 = img2.load()
    pixels_diff = diff_img.load()

    for x in range(img1.width):
        for y in range(img1.height):
            pixels_diff[x , y ] = (255, 255, 255)

    draw.rectangle([(0, 0), diff_img.size], outline="red", width=border_width)

    for x in range(img1.width):
        for y in range(img1.height):
            if pixels1[x, y] != pixels2[x, y]:
                draw.rectangle(
                    [(x  - 1, y  - 1), (x  + 1, y  + 1)],
                    outline=(255, 128, 128),
                    width=3
                )

    for x in range(img1.width):
        for y in range(img1.height):
            if pixels1[x, y] != pixels2[x, y]:
                pixels_diff[x , y] = (0,0,0)

    diff_img.save(output_path)

from PIL import Image

def non_white_to_red(image_path):
    img = Image.open(image_path).convert("RGB")
    
    pixels = img.load()
    
    for x in range(img.width):
        for y in range(img.height):
            if pixels[x, y] != (255, 255, 255): 
                pixels[x, y] = (255, 0, 0) 
    
    img.save(image_path.split('.p')[0] + '_mod.png')

def show_difference(image_path, image2_path):
    img = Image.open(image_path).convert("RGB")
    img2 = Image.open(image2_path).convert("RGB")
    
    pixels = img.load()
    pixels2 = img2.load()
    
    for x in range(img.width):
        for y in range(img.height):
            if pixels2[x, y] != (255, 255, 255): 
                pixels[x, y] = pixels2[x, y]
    
    img.save(image_path.split('.p')[0] + '_diff.png')

parser = argparse.ArgumentParser()
parser.add_argument('stage')
args = parser.parse_args()
stage = args.stage

image1_path = f'../maps/{stage}/map.png'
image2_path = f'../correct_maps/{stage}/c_map.png'
output_path = f'../maps/{stage}/difference_map.png'
non_white_to_red(image1_path)
non_white_to_red(image2_path)
compare_images(image1_path, image2_path, output_path)
show_difference(image1_path, output_path)


image1_path = f'../maps/{stage}/inflated_map.png'
image2_path = f'../correct_maps/{stage}/c_inflated_map.png'
output_path = f'../maps/{stage}/difference_inflated_map.png'
non_white_to_red(image1_path)
non_white_to_red(image2_path)
compare_images(image1_path, image2_path, output_path)
show_difference(image1_path, output_path)