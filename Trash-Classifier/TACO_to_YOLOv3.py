"""
    The TACO data set is presented in .json file types that roughly follow the COCO formatting.
    
    This script will take the TACO dataset annotations as it is presented in April 2021 and convert it into
    the same type of formatting and image size that is presented in the tutorial for the YOLO V3 training
    
    This file was ran three times manually changing path names and the resulting text file name for training, 
    testing and validation
"""

import json
import os
from PIL import Image
from yolov3.configs import *

#path to TACO data annotation file
path_to_json_annotations = 'C:/B_Folder/Desktop/Education/WPI/RBE_594_Capstone_Experience/Trash_Detection/Code_Research/TACO-master/data/annotations_0_val.json'
#path to TACO data image file
path_to_images           = 'C:/B_Folder/Desktop/Education/WPI/RBE_594_Capstone_Experience/Trash_Detection/Code_Research/TACO-master/data/'
#save path for resized images
path_save_images         = './val/'

#Image_Size_Final = TRAIN_INPUT_SIZE
Image_Size_Final = 416

Img_Base_Counter = 1

#Open Text File
Text_File = open('TACO_val.txt', 'w')

# read in .json format
with open(path_to_json_annotations,'rb') as file:
    doc = json.load(file)
    
# get annotations
annotations = doc['annotations']
image_data  = doc['images']

for img in image_data:

    img_id     = img['id']
    img_path   = img['file_name']
    img_width  = img['width']
    img_height = img['height']
    
    anno_str = ''
    
    for anno in annotations:
        #only add annotations for the image that is selected
        if img_id == anno['image_id']:
            #Convert Bounding Box Points into percentile locations and then multiply by final image size for adjustment
            xmin = int( ( anno['bbox'][0] / img_width  ) * Image_Size_Final )
            ymin = int( ( anno['bbox'][1] / img_height ) * Image_Size_Final )
            xmax = int( ( (anno['bbox'][0] + anno['bbox'][2]) / img_width  ) * Image_Size_Final )
            ymax = int( ( (anno['bbox'][1] + anno['bbox'][3]) / img_height ) * Image_Size_Final )
            
            #error checking that the bounds are being set outside the image size
            if xmin < 0:
                xmin = 0
            if ymin < 0:
                ymin = 0
            if xmax > Image_Size_Final:
                xmax = Image_Size_Final
            if ymax > Image_Size_Final:
                ymax = Image_Size_Final
            
            #look up object category id
            cat_id = anno['category_id']
            
            #generate annotations string in yolo formatting
            anno_str += ' ' + str(xmin) + ',' + str(ymin) + ',' + str(xmax) + ',' + str(ymax) + ',' + str(cat_id)
                
    im = Image.open( path_to_images + img_path)
    
    #resize taco images to appropriate size for yolo training
    im = im.resize((Image_Size_Final, Image_Size_Final))
    full_save_path = path_save_images + 'Trash_Img_' + str(Img_Base_Counter) + '.jpg'
    Img_Base_Counter += 1
    im.save(full_save_path)
      
    Text_File.writelines(full_save_path + anno_str + '\n')
    
    
Text_File.close()