#!/usr/bin/env python3
import torch
from torchvision import transforms
from yolact import Yolact
from data import cfg, set_cfg
import cv2
import torch.backends.cudnn as cudnn
import time
import rospy
from std_msgs.msg import String, UInt16MultiArray

model_path = "/home/martin/catkin_ws/src/yumi_controller/scripts/detection/weights/example_base_25_290909.pth"
config = "example_base_config"


# Detekcia objektov
def detect_objects(frame):
    with torch.no_grad():
        img_tensor = transforms.ToTensor()(frame).cuda().unsqueeze(0)
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
        target_class = "Nepoznam objekt"

    threshold = 0.7  # prah pre skóre detekcie
    
    try:
        # Detekcia objektov
        classes, scores, boxes, masks = detect_objects(frame)
        
        # Overenie prítomnosti cieľového objektu
        state, pos = get_pos(boxes, classes, target_class, frame.shape, threshold)
        if state:
            pub.publish(pos)
            rospy.loginfo(f"{target_class} je prítomný na snímke.")
        else:
            rospy.loginfo(f"{target_class} nie je prítomný na snímke.")
            
    except TypeError:
        rospy.loginfo("Chyba pri spracovaní snímky.")


# Vypocet stredu objektu
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
    # Nastavenie kamery
    cap = cv2.VideoCapture(4)

    start_time = time.time()
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        try:
            # Detekcia objektov
            classes, scores, boxes, masks = detect_objects(frame)

            for i in range(len(classes)):
                class_name = cfg.dataset.class_names[classes[i]]
                score = scores[i]
                box = boxes[i]
                
                if score > 0.7:  #definovanie zhody
                    x1, y1, x2, y2 = int(box[0] * frame.shape[1]), int(box[1] * frame.shape[0]), int(box[2] * frame.shape[1]), int(box[3] * frame.shape[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name}: {score:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
        except TypeError:
            pass
        
        # Zobrazenie snimkov za sekundu
        fps_end_time = time.time()
        fps_elapsed_time = fps_end_time - start_time
        fps = frame_count / fps_elapsed_time if fps_elapsed_time > 0 else 0

        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('YOLACT Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Kalkulacia FPS
        if frame_count >= 20:
            frame_count = 0
            start_time = time.time()

    cap.release()
    cv2.destroyAllWindows()