##Make Data
#
#take background picture
#choose random number of objects between MIN_OBJ_PLACED and MAX_OBJ_PLACED
#paste objects to background and monitor/log their location and size
#save new picture
#save picture location and each object bbox(xmin,ymin,xmAx,ymAx) and class number to txt file
#
# print("xxx :  ",xxx)

from PIL import Image
import os
import numpy as np
import cv2
#from random import randrange, randint

Total_Pics = 25 #Total Number of Pictures to Generate

Path_Background = 'C:/Users\Bee/Desktop/_Background Items/'
Path_Objects    = 'C:/Users/Bee/Desktop/taco_obj/'
Path_Save_Loc   = 'C:/B_Folder/Desktop/Education/WPI/RBE_594_Capstone_Experience/Trash_Detection/Code_Research/TensorFlow-2.x-YOLOv3-master/train_testingDataGen/'

Generic_Img_Name = 'Generated_Trash_Img_'
Img_Format       = '.jpg'

Images_Backgound = [Path_Background+img for img in os.listdir(Path_Background) if (img.endswith(".jpg") or img.endswith(".png"))]
Objects_SubDir   = [Path_Objects+SubDir+"/" for SubDir in os.listdir(Path_Objects)]
#print("Objects_SubDir :  ", Objects_SubDir)

#Images_Objects   = [img for img in os.listdir(Objects_SubDir) if (img.endswith(".jpg") or img.endswith(".png"))]
Images_Objects   = []

for directory in Objects_SubDir:
    Sub_Obj_List = [directory+img for img in os.listdir(directory) if (img.endswith(".jpg") or img.endswith(".png"))]
    Images_Objects.extend(Sub_Obj_List)
    
#print("\n\nImages_Objects :  ", Images_Objects)
#print("\n\n")

MIN_OBJ_PLACED = 3
MAX_OBJ_PLACED = 9

MAX_OBJ_COUNT = len(Images_Objects)

SIZE_IMG_OUTPUT = 416
MAX_PERCENT_OBJ_SIZE = 0.4

#resize_percentage_list
RESIZE_PERCENTAGE_LIST = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]

MAX_OBJ_SIZE = SIZE_IMG_OUTPUT * MAX_PERCENT_OBJ_SIZE

#dictionary matching object class name and id
OBJ_CLASS_ID = {
    "Aluminium_Foil" : 0,
    "Battery" : 1,
    "Aluminium_Blister_Pack" : 2,
    "Carded_Blister_Pack" : 3,
    "Other_Plastic_Bottle" : 4,
    "Clear_Plastic_Bottle" : 5,
    "Glass_Bottle" : 6,
    "Plastic_Bottle_Cap" : 7,
    "Metal_Bottle_Cap" : 8,
    "Broken_Glass" : 9,
    "Food_Can" : 10,
    "Aerosol" : 11,
    "Drink_Can" : 12,
    "Toilet_Tube" : 13,
    "Other_Carton" : 14,
    "Egg_Carton" : 15,
    "Drink_Carton" : 16,
    "Corrugated_Carton" : 17,
    "Meal_Carton" : 18,
    "Pizza_Box" : 19,
    "Paper_Cup" : 20,
    "Disposable_Plastic_Cup" : 21,
    "Foam_Cup" : 22,
    "Glass_Cup" : 23,
    "Other_Plastic_Cup" : 24,
    "Food_Waste" : 25,
    "Glass_Jar" : 26,
    "Plastic_Lid" : 27,
    "Metal_Lid" : 28,
    "Other_Plastic" : 29,
    "Magazine_Paper" : 30,
    "Tissues" : 31,
    "Wrapping_Paper" : 32,
    "Normal_Paper" : 33,
    "Paper_Bag" : 34,
    "Plastified_Paper_Bag" : 35,
    "Plastic_Film" : 36,
    "Six_Pack_Rings" : 37,
    "Garbage_Bag" : 38,
    "Other_Plastic_Wrapper" : 39,
    "Single-Use_Carrier_Bag" : 40,
    "Polypropylene_Bag" : 41,
    "Crisp_Packet" : 42,
    "Spread_Tub" : 43,
    "Tupperware" : 44,
    "Disposable_Food_Container" : 45,
    "Foam_Food_Container" : 46,
    "Other_Plastic_Container" : 47,
    "Plastic_Glooves" : 48,
    "Plastic_Utensils" : 49,
    "Pop_Tab" : 50,
    "Rope_&_Strings" : 51,
    "Scrap_Metal" : 52,
    "Shoe" : 53,
    "Squeezable_Tube" : 54,
    "Plastic_Straw" : 55,
    "Paper_Straw" : 56,
    "Styrofoam_Piece" : 57,
    "Unlabeled_Litter" : 58,
    "Cigarette" : 59,
    "Place_Holder_1" : 60,
    "Place_Holder_2" : 61,
    "Place_Holder_3" : 62,
    "Place_Holder_4" : 63,
    "Place_Holder_5" : 64,
    "Place_Holder_6" : 65,
    "Place_Holder_7" : 66,
    "Place_Holder_8" : 67,
    "Place_Holder_9" : 68,
    "Place_Holder_10" : 69    
    }

#Open Text File
Text_File = open('Generated_Data_Train.txt', 'w')

#masking values
threshold=225
dist=6

#counter for potential statistics
total_num_Objects_placed = 0

for i in range(Total_Pics):
    
    #Open Background Image and Resize to standardized sized for yolo training
    img_primary = Image.open(Images_Backgound[i])
    img_primary = img_primary.resize((SIZE_IMG_OUTPUT, SIZE_IMG_OUTPUT))
    
    #Generate a random number of objects to be placed in the background image
    rand_num_objs_placed = np.random.randint(MIN_OBJ_PLACED, MAX_OBJ_PLACED)
    
    Annotations = ""
    
    Objects_List = []
    Objects_Max_Dim_List = []
    #Generate List of object images to be pasted into a background image
    for q in range(rand_num_objs_placed):
        
        #Randomly select an object from the list to be added
        rand_obj_num = np.random.randint(0,MAX_OBJ_COUNT)
        
        #Find and Open Object Image
        img_obj = Image.open(Images_Objects[rand_obj_num]).convert('RGBA')
                
        #Randomly Rotate the input object image
        rand_rot = np.random.randint(0,360)
        img_obj = img_obj.rotate(rand_rot, expand = True)
        
    #*temp resizing of img_obj
        img_width, img_height = img_obj.size
        
        #use bigger dimension to generate resizing percentage to create baseline size
        if img_width >= (img_height):            
            standardized_sizing = MAX_OBJ_SIZE/img_width            
        else:
            standardized_sizing = MAX_OBJ_SIZE/img_height                
        
        #generate standard width and height of object image
        standardized_width  = int(standardized_sizing * img_width)
        standardized_height = int(standardized_sizing * img_height)
        
        new_size = (standardized_width, standardized_height)
        img_obj = img_obj.resize(new_size)
        
    #*temp resize over
        
        img_width, img_height = img_obj.size
        
        #Randomly resize object image from standard (will be smaller or close to same)
        random_resize = np.random.randint(0, len(RESIZE_PERCENTAGE_LIST))       
        
        #random_resize_percentage = RESIZE_PERCENTAGE_LIST[random_resize]
        random_resize_percentage = np.random.uniform(0.05, 1.0)
        
        new_width = int(random_resize_percentage * img_width)
        new_height = int(random_resize_percentage * img_height)
        
        new_size = (new_width, new_height)
        img_obj = img_obj.resize(new_size)
        
        #print("\n\nBEFORE Rotation Image Width, Image Height :   ", new_size)
        
        #track object size for annotations lists
        xmin = np.random.randint(0, (SIZE_IMG_OUTPUT-new_width))
        ymin = np.random.randint(0, (SIZE_IMG_OUTPUT-new_height))
        
        xmax = xmin + new_width
        ymax = ymin + new_height
        
        #save relevant object data into list so it can be pasted by largest to smallest
        #this is to alleviate issues with a smaller image being pasted into an image and
        #and then subsequently having a larger object pasted over it and effectively "erasing" the object image
        
        Objects_List.append(list((img_obj, xmin, ymin, xmax, ymax, rand_obj_num)) )
        Objects_Max_Dim_List.append(max(new_width, new_height))
        
        
    for q in range(rand_num_objs_placed):
        #Find max dim image index and pull out relevant data from list and removing it from the list for next iteration
        Max_Obj_Index = Objects_Max_Dim_List.index(max(Objects_Max_Dim_List))
        max_dim = Objects_Max_Dim_List.pop(Max_Obj_Index)
        obj_list = Objects_List.pop(Max_Obj_Index)
        
        img_obj      = obj_list[0]
        xmin         = obj_list[1]  
        ymin         = obj_list[2]  
        xmax         = obj_list[3]  
        ymax         = obj_list[4]  
        rand_obj_num = obj_list[5]  
        
        img_spot = (xmin, ymin)
        #Paste Objects into background from largest to smallest
        #Convert White-Space in Obj Image into mask
        arr=np.array(np.asarray(img_obj))
        r,g,b,a=np.rollaxis(arr,axis=-1)
        
        #mask out pixels that are exactly white
        #mask=((r==255)&(g==255)&(b==255))
        
        #mask out pixels that are white-ish
        mask=((r>threshold)
              & (g>threshold)
              & (b>threshold)
              & (np.abs(r-g)<dist)
              & (np.abs(r-b)<dist)
              & (np.abs(g-b)<dist)
              )
        
        
        arr[mask,3]=0
        mask_img=Image.fromarray(arr,mode='RGBA')
        
        #Paste Object image into background image
        img_primary.paste(img_obj , img_spot, mask_img)   # **************** img obj and spot need to be pulled from list
        
        #Determine Object Class by parsing Object Folder
        parse_obj_path = Images_Objects[rand_obj_num].split('/')
        
        #print("parse_obj_path[5] :  ",parse_obj_path[5])
        #print("OBJ_CLASS_ID[parse_obj_path[5]] :  ",OBJ_CLASS_ID[parse_obj_path[5]])
        
        #Generate YOLO Annotations list for image
        Annotations += Annotations + " " + str(xmin) + "," + str(xmin) + "," + str(xmin) + "," + str(xmin) + "," + str(OBJ_CLASS_ID[parse_obj_path[5]])
        
        #Monitor Number of Objects Placed in Background Images ... for stats
        total_num_Objects_placed += 1


    #save the generated image
    Path_Save = Path_Save_Loc + Generic_Img_Name + str(i) + Img_Format
    img_primary.save(Path_Save)
    
    #Write annotations and path to file for use in training
    Text_File.writelines(Path_Save + Annotations + '\n')
    
Text_File.close()

print("\n\n", total_num_Objects_placed, " objects(s) were placed in ", Total_Pics, " background images\n")