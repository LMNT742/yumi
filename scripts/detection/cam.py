#!/usr/bin/env python3
import torch
from torchvision import transforms
from yolact import Yolact
from data import cfg, set_cfg
import cv2
import torch.backends.cudnn as cudnn
import time
import rospy
from std_msgs.msg import String, UInt16MultiArray, Int8

model_path = "/home/martin/catkin_ws/src/yumi_controller/scripts/detection/weights/example_base_25_290909.pth"
config = "example_base_config"
threshold = 0.7

# Object detection
def detect_objects(frame):
    with torch.no_grad():
        img_tensor = transforms.ToTensor()(frame).cuda().unsqueeze(0)
        preds = net(img_tensor)
        
        classes = preds[0]["detection"]["class"]
        scores = preds[0]["detection"]["score"]
        boxes = preds[0]["detection"]["box"]
        masks = preds[0]["detection"]["mask"]
        
        return classes, scores, boxes, masks
    
    
# Callback definition for subscriber
def voice_callback(msg: String):
    target_class_list = ["Hammer", "Pliers", "Crossheaded_Screwdriver", "Flat_Screwdriver", "Wrench"]
    received_bytes = msg.data.encode('utf-8')
    decoded_text = received_bytes.decode('utf-8')
    rospy.loginfo(decoded_text)

    # Set class for detection
    if decoded_text == "kľúč":
        target_class = "Wrench"
    elif decoded_text == "kladivo":
        target_class = "Hammer"
    elif decoded_text == "kliešte":
        target_class = "Pliers"
    elif decoded_text == "krížový skrutkovač":
        target_class = "Crossheaded_Screwdriver"
    elif decoded_text == "plochý skrutkovač":
        target_class = "Flat_Screwdriver"
    elif decoded_text == "ukonči prevádzku":
        rospy.signal_shutdown("Ukoncenie programu")
    else:
        target_class = "Nepoznam objekt"
    
    if status == 0:
        if target_class in target_class_list:
            try:        
                # Searching for selected equipment
                state, pos, orientation = get_pos(target_class, threshold)
                if state:
                    pub_coords.publish(pos)
                    pub_target.publish(target_class + "/" + orientation)
                    rospy.loginfo(f"{target_class} je prítomný na snímke.")
                else:
                    rospy.loginfo(f"{target_class} nie je prítomný na snímke.")
                    
            except TypeError:
                rospy.loginfo(f"{target_class} nie je prítomný na snímke.")
        else:
            rospy.loginfo("Na snímke sa nenachádza detekovateľný objekt!")
    elif status == 1:
        rospy.loginfo("Yumi musí najskôr ukončiť úlohu!")


# Mid bbox counting
def get_pos(target_class, threshold):
    orientation = ""
    pos_list = []
    pos_classes, pos_scores, pos_boxes, pos_masks = detect_objects(frame)
   
    for i in range(len(pos_classes)):
        class_name = cfg.dataset.class_names[pos_classes[i]]
        score = pos_scores[i]
        box = pos_boxes[i]
        x_1, y_1, x_2, y_2 = int(box[0] * frame.shape[1]), int(box[1] * frame.shape[0]), int(box[2] * frame.shape[1]), int(box[3] * frame.shape[0])

        width = x_2 - x_1
        height = y_2 - y_1
        
        if class_name == target_class and score > threshold:
            center_x = (x_1 + x_2) // 2
            center_y = (y_1 + y_2) // 2
            # Vertical orientation
            if width < height:
                orientation = "up"
            # Horizontal orientation
            elif height < width:
                orientation = "right"

            pos_list.append((center_x, center_y))

    if pos_list:
        list_of_pos = UInt16MultiArray()
        list_of_pos.data = pos_list[0]
        return True, list_of_pos, orientation
    else:
        return False, None, orientation


def status_callback(msg: Int8):
    global status
    status = msg.data


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
    # Subscriber/Publisher definition
    rospy.init_node("object_detection")
    sub_commands = rospy.Subscriber("/voice_reg", String, voice_callback)
    sub_yumi_status = rospy.Subscriber("/yumi_status", Int8, status_callback)
    pub_coords = rospy.Publisher("/object_reg", UInt16MultiArray, queue_size=10)
    pub_target = rospy.Publisher("/get_object_class", String, queue_size=10)
    # Camera setup
    cap = cv2.VideoCapture(4) # NTBK = 0, 4 = astra

    start_time = time.time()
    frame_count = 0

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        try:
            classes, scores, boxes, masks = detect_objects(frame)

            for i in range(len(classes)):
                class_name = cfg.dataset.class_names[classes[i]]
                score = scores[i]
                box = boxes[i]
                # Displaying bboxes
                if score > threshold:
                    x1, y1, x2, y2 = int(box[0] * frame.shape[1]), int(box[1] * frame.shape[0]), int(box[2] * frame.shape[1]), int(box[3] * frame.shape[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name}: {score:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
        except TypeError:
            pass
        
        # FPS counting
        fps_end_time = time.time()
        fps_elapsed_time = fps_end_time - start_time
        fps = frame_count / fps_elapsed_time if fps_elapsed_time > 0 else 0

        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('YOLACT Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if frame_count >= 20:
            frame_count = 0
            start_time = time.time()

    cap.release()
    cv2.destroyAllWindows()