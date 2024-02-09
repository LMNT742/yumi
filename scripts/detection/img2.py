import cv2
import torch
from torchvision import transforms
from yolact import Yolact
from data import set_cfg, cfg
import torch.backends.cudnn as cudnn

model_path = "/home/martin/catkin_ws/src/yolact_ros/scripts/yolact/weights/example_base_25_290909.pth"
config = "example_base_config"

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

# Funkce pro detekci objektů z obrázku
def detect_objects_from_image(image_path):
    with torch.no_grad():
        image = cv2.imread(image_path)
        img_tensor = transforms.ToTensor()(image).cuda().unsqueeze(0)
        preds = net(img_tensor)
        
        classes = preds[0]["detection"]["class"]
        scores = preds[0]["detection"]["score"]
        boxes = preds[0]["detection"]["box"]
        masks = preds[0]["detection"]["mask"]
        
        return classes, scores, boxes, masks
    

# Cesta k obrázku
image_path = '/home/martin/catkin_ws/src/yolact_ros/scripts/yolact/tools.jpg'

# Detekce objektů z obrázku
classes, scores, boxes, masks = detect_objects_from_image(image_path)

# Vykreslení detekcí na obrázek
image = cv2.imread(image_path)

for i in range(len(classes)):
    class_name = cfg.dataset.class_names[classes[i]]
    score = scores[i]
    box = boxes[i]
    
    if score > 0.95:  # Přidejte filtraci detekcí na základě skóre (změňte prahovou hodnotu podle potřeby)
        x1, y1, x2, y2 = int(box[0] * image.shape[1]), int(box[1] * image.shape[0]), int(box[2] * image.shape[1]), int(box[3] * image.shape[0])
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name}: {score:.2f}"
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Zobrazení výsledného obrázku s detekcemi
cv2.imshow('Detected Objects', image)
cv2.waitKey(0)
cv2.destroyAllWindows()