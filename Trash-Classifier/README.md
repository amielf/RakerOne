# TensorFlow-2.x-YOLOv3 and YOLOv4 from Python Lessons
​
YOLOv3 and YOLOv4 implementation in TensorFlow 2.x, with support for training, transfer training, object tracking mAP and so on...
​
This code is 90% from Python Lessons Github and tutorial that can be found on YouTube.
If you want to do anything elaborate with the YOLO portion of this code, you are better off going to that source
and starting from scratch and not trying to recycle code from someone else's tutorial.
​
The ORIGINAL YOLO implemenations can be found here:
https://github.com/pythonlessons/TensorFlow-2.x-YOLOv3
​
YouTube Tutorial:
https://www.youtube.com/watch?v=j-CfVrhR4Zo&list=PLbMO9c_jUD473OgrKYYMLRMEz-XZjG--n&index=6
​
​
## Quick start
Start with using pretrained weights to test predictions on image
```
python detection_custom.py
```
​
​
​
## Quick training for custom TACO dataset
Now, you can train it and then evaluate your model
```
python train.py
tensorboard --logdir=log
```
​
#Data
The TACO dataset was gotten from here:
http://tacodataset.org/
​
Follow the directions that they give to download their data set.
​
#Folder Contents:
* checkpoints: 			TACO training save points
* deep_sort: 				original supporting code files
* demo_start: 			output from original yolo implementation
* images: 				empty (contained sample demo images for original yolo implementation)
* log:					training log data
* mAP:					ground truth files for calculating mAP
* model_data: 			original COCO weights for yolo v3 and v4
* outputs:				outputs from demonstration_custom.py
* test:					testing image data 
* tools:					original data conversion files (they dont work for converting TACO to YOLO)
* train:					training image data
* train_testingDataGen:	output images for attempts at training data generation
* trash_demo_pics:		random input images for demonstraiting effectiveness of trained network
* val:					validation image data
* yolov3:					original supporting code files
