import cv2
import numpy as np

from openpilot.selfdrive.modeld.runners.onnxmodel import create_ort_session

# TODO: use the names metadata from the onnx model to avoid this huge list
CLASSES = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

def xywh2xyxy(x):
  y = x.copy()
  y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
  y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
  y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
  y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
  return y

def non_max_suppression(boxes, scores, threshold):
  assert boxes.shape[0] == scores.shape[0]
  ys1 = boxes[:, 0]
  xs1 = boxes[:, 1]
  ys2 = boxes[:, 2]
  xs2 = boxes[:, 3]
  areas = (ys2 - ys1) * (xs2 - xs1)
  scores_indexes = scores.argsort().tolist()
  boxes_keep_index = []
  while len(scores_indexes):
    index = scores_indexes.pop()
    boxes_keep_index.append(index)
    if not len(scores_indexes):
      break
    ious = compute_iou(boxes[index], boxes[scores_indexes], areas[index],
            areas[scores_indexes])
    filtered_indexes = set((ious > threshold).nonzero()[0])
    scores_indexes = [
      v for (i, v) in enumerate(scores_indexes)
      if i not in filtered_indexes
    ]
  return np.array(boxes_keep_index)

def compute_iou(box, boxes, box_area, boxes_area):
  assert boxes.shape[0] == boxes_area.shape[0]
  ys1 = np.maximum(box[0], boxes[:, 0])
  xs1 = np.maximum(box[1], boxes[:, 1])
  ys2 = np.minimum(box[2], boxes[:, 2])
  xs2 = np.minimum(box[3], boxes[:, 3])
  intersections = np.maximum(ys2 - ys1, 0) * np.maximum(xs2 - xs1, 0)
  unions = box_area + boxes_area - intersections
  ious = intersections / unions
  return ious

def nms(prediction, conf_thres=0.3, iou_thres=0.45):
  prediction = prediction[prediction[...,4] > conf_thres]
  boxes = xywh2xyxy(prediction[:, :4])
  res = non_max_suppression(boxes,prediction[:,4],iou_thres)
  result_boxes = []
  result_scores = []
  result_classes = []
  for r in res:
    result_boxes.append(boxes[r])
    result_scores.append(prediction[r,4])
    result_classes.append(np.argmax(prediction[r,5:]))
  return np.c_[result_boxes, result_scores, result_classes]

class YoloRunner:
  def __init__(self, onnx_path):
    self.sess = create_ort_session(onnx_path)

  # TODO: add crop to center
  def preprocess_image(self, img):
    img = cv2.resize(img, (640, 640))
    img = np.expand_dims(img, 0).astype(np.float32)
    img = img.transpose(0, 3, 1, 2)
    img /= 255
    return img

  def run(self, img):
    img = self.preprocess_image(img)
    res = self.sess.run(None, {'image': img})
    res = nms(res[0])
    return [{"pred_class": CLASSES[int(opt[-1])], "prob": opt[-2], "pt1": opt[:2].astype(int).tolist(), "pt2": opt[2:4].astype(int).tolist()} for opt in res]