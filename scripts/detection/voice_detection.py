#!/usr/bin/env python3
import torch
from torchvision import transforms
from yolact import Yolact
from data import cfg, set_cfg
import cv2
import torch.backends.cudnn as cudnn
import rospy
from std_msgs.msg import String, UInt16MultiArray
import sys

model_path = "/home/martin/catkin_ws/src/yumi_controller/scripts/detection/weights/example_base_25_290909.pth"
config = "example_base_config"
image_path = '/home/martin/catkin_ws/src/yumi_controller/scripts/detection/tool.jpg'


# Detekcia objektov
def detect_objects(open_image):
    with torch.no_grad():
        img_tensor = transforms.ToTensor()(open_image).cuda().unsqueeze(0)
        preds = net(img_tensor)
        
        classes = preds[0]["detection"]["class"]
        scores = preds[0]["detection"]["score"]
        boxes = preds[0]["detection"]["box"]
        masks = preds[0]["detection"]["mask"]
        
        return classes, scores, boxes, masks
    
    
# Definicia callbacku pre subscribera
def voice_callback(msg: String):
    received_bytes = msg.data.encode('utf-8')
    decoded_text = received_bytes.decode('utf-8')
    rospy.loginfo(decoded_text)

    if decoded_text == "koniec":
        rospy.signal_shutdown('Ukončenie programu')
        sys.exit(0)

    if decoded_text == "kľúč":
        target_class = "Wrench"  # nastavte cieľovú triedu
    elif decoded_text == "kladivo":
        target_class = "Hammer"
    elif decoded_text == "kliešte":
        target_class = "Pliers"
    elif decoded_text == "krížový skrutkovač":
        target_class = "Crossheaded_Screwdriver"
    elif decoded_text == "plochý skrutkovač":
        target_class = "Flat_Screwdriver"
    else:
        target_class = "Unknown tool"
    threshold = 0.7  # prah pre skóre detekcie
    
    try:
        # Detekcia objektov
        classes, scores, boxes, masks = detect_objects(image)
        
        # Overenie prítomnosti cieľového objektu
        state, pos = get_pos(boxes, classes, target_class, image.shape, threshold)
        if state:
            pub.publish(pos)
            rospy.loginfo(f"{target_class} je prítomný na snímke.")
        else:
            rospy.loginfo(f"{target_class} nie je prítomný na snímke.")
            
    except TypeError as e:
        rospy.loginfo("Chyba pri spracovaní snímky.", e)


def get_pos(boxes, classes, target_class, frame_shape, threshold):
    pos_list = []
    
    for i in range(len(classes)):
        class_name = cfg.dataset.class_names[classes[i]]
        score = scores[i]
        box = boxes[i]
        
        if class_name == target_class and score > threshold:  # Ak nájdeme cieľový objekt
            x1, y1, x2, y2 = int(box[0] * frame_shape[1]), int(box[1] * frame_shape[0]), int(box[2] * frame_shape[1]), int(box[3] * frame_shape[0])
            center_x = int(x1 + x2) // 2
            center_y = (y1 + y2) // 2
            pos_list.append((center_x, center_y))

    if pos_list:
        list_of_pos = UInt16MultiArray()
        list_of_pos.data = pos_list[0]
        return True, list_of_pos  # Ak máme pozície, vrátime prvú pozíciu
    else:
        return False, None
    

def plot_bboxes(image, boxes, labels=[], score=True, conf=None):
    #Define COCO Labels

    colors=(128, 128, 128)
    #plot each boxes
    for box in boxes:
    #add score in label if score=True
        if score :
            label = labels[int(box[-1])+1] + " " + str(round(100 * float(box[-2]),1)) + "%"
        else :
            label = labels[int(box[-1])+1]
        #filter every box under conf threshold if conf threshold setted
        if conf :
            if box[-2] > conf:
                color = colors[int(box[-1])]
                box_label(image, box, label, color)
        else:
            color = colors[int(box[-1])]
            box_label(image, box, label, color)


def box_label(image, box, label='', color=(128, 128, 128), txt_color=(255, 255, 255)):
    text_str = label

    font_face = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.6
    font_thickness = 1

    text_w, text_h = cv2.getTextSize(text_str, font_face, font_scale, font_thickness)[0]

    text_pt = (x1, y1 - 3)
    text_color = [255, 255, 255]

    cv2.rectangle(image, (x1, y1), (x1 + text_w, y1 - text_h - 4), color, -1)
    cv2.putText(image, text_str, text_pt, font_face, font_scale, text_color, font_thickness, cv2.LINE_AA)


with torch.no_grad():
    cudnn.benchmark = True
    cudnn.fastest = True
    torch.set_default_tensor_type('torch.cuda.FloatTensor')   

    print('Loading model from', model_path)
    set_cfg(config)
    net = Yolact()
    net.load_weights(path = model_path)
    net.eval()
    print('Done.')

    net = net.cuda()
    net.detect.use_fast_nms = True
    cfg.mask_proto_debug = False

if __name__ == "__main__":
    # definovanie subscribera
    rospy.init_node("object_detection")
    sub = rospy.Subscriber("/voice_reg", String, voice_callback)
    pub = rospy.Publisher("/object_reg", UInt16MultiArray, queue_size=10)

    # Load picture
    img = cv2.imread(image_path)

    # Detekce objektů z obrázku
    classes, scores, boxes, masks = detect_objects(img)

    for i in range(len(classes)):
        class_name = cfg.dataset.class_names[classes[i]]
        score = scores[i]
        box = boxes[i]
        
        if score > 0.7:  # Přidejte filtraci detekcí na základě skóre (změňte prahovou hodnotu podle potřeby)
            x1, y1, x2, y2 = int(box[0] * img.shape[1]), int(box[1] * img.shape[0]), int(box[2] * img.shape[1]), int(box[3] * img.shape[0])
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}: {score:.2f}"
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                #show image
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    cv2.imshow("Detection",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()