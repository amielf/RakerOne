#================================================================
#
#   File name   : detection_custom.py
#   Author      : PyLessons
#   Created date: 2020-09-17
#   Website     : https://pylessons.com/
#   GitHub      : https://github.com/pythonlessons/TensorFlow-2.x-YOLOv3
#   Description : object detection image and video example
#
#================================================================
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import cv2
import numpy as np
import tensorflow as tf
from yolov3.utils import detect_image, detect_realtime, detect_video, Load_Yolo_model, detect_video_realtime_mp
from yolov3.configs import *
import imageio
from PIL import Image

image_folder_1 = 'trash_demo_pics'
image_folder_2 = 'test'
save_folder  = 'outputs/yolo_v4/'
video_name   = 'outputs/yolo_v4/Yolo_V4_Trash_Detection_Demo.mp4'

fps = 2
display_img_time = 3 #this is the number of SECONDS to display each detected image output in the video


images_1 = [img for img in os.listdir(image_folder_1) if (img.endswith(".jpg") or img.endswith(".png"))]
images_2 = [img for img in os.listdir(image_folder_2) if (img.endswith(".jpg") or img.endswith(".png"))]

fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter(video_name, fourcc, fps, (YOLO_INPUT_SIZE, YOLO_INPUT_SIZE))
img_counter = 1


writer = imageio.get_writer(video_name, format='mp4', mode='I', fps=fps)

yolo = Load_Yolo_model()

for image in images_1:
    
    image_path = os.path.join(image_folder_1, image)
    
    img_detected = detect_image(yolo, image_path, save_folder + "trash_detect_demo_"+str(img_counter)+".jpg", input_size=YOLO_INPUT_SIZE, show=False, CLASSES=TRAIN_CLASSES, rectangle_colors=(255,0,0))
    img_detected = cv2.cvtColor(img_detected, cv2.COLOR_BGR2RGB)
    img_detected = Image.fromarray(img_detected)
    img_detected = img_detected.resize((YOLO_INPUT_SIZE,YOLO_INPUT_SIZE))
    
    img_counter += 1
    #Display Image for 3 seconds at 1 fps and move to next image
    for i in range(display_img_time):
        
        writer.append_data(np.array(img_detected))

for image in images_2:
    
    image_path = os.path.join(image_folder_2, image)
    
    img_detected = detect_image(yolo, image_path, save_folder + "trash_detect_demo_"+str(img_counter)+".jpg", input_size=YOLO_INPUT_SIZE, show=False, CLASSES=TRAIN_CLASSES, rectangle_colors=(255,0,0))
    img_detected = cv2.cvtColor(img_detected, cv2.COLOR_BGR2RGB)
    img_detected = Image.fromarray(img_detected).resize((YOLO_INPUT_SIZE,YOLO_INPUT_SIZE))
    
    img_counter += 1
    #Display Image for 3 seconds at 1 fps and move to next image
    for i in range(display_img_time):
        
        writer.append_data(np.asarray(img_detected))
    
cv2.destroyAllWindows()
video.release()
writer.close()


