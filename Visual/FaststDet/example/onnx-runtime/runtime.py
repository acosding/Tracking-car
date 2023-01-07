import os
import time

import cv2
import numpy as np


# sigmoid函数
def sigmoid(x):
    return 1.0 / (1 + np.exp(-x))


# tanh函数
def tanh(x):
    return 2.0 / (1 + np.exp(-2 * x)) - 1


def draw_pred(frame, class_name, conf, left, top, right, bottom):
    """
    绘制预测结果
    """
    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
    label = f"{class_name}: {conf:.2f}"
    labelSize, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    top = max(top - 10, labelSize[1])
    left = max(left, 0)
    cv2.putText(
        frame,
        label,
        (left, top),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        thickness=2,
    )


class FastestDetOnnx:
    def __init__(self, confThreshold=0.5, nmsThreshold=0.45, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        import onnxruntime

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "FastestDet.onnx")
        self.classes = list(map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 500
        self.inpHeight = 500
        self.session = onnxruntime.InferenceSession(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.drawOutput = drawOutput

    def __preprocess(self, src_img, size):
        """
        前处理, 对输入图像进行归一化
        """
        output = cv2.resize(src_img, (size[0], size[1]), interpolation=cv2.INTER_AREA)
        output = output.transpose(2, 0, 1)
        output = output.reshape((1, 3, size[1], size[0])) / 255

        return output.astype("float32")

    def __postprocess(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        outs = outs.transpose(1, 2, 0)
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        feature_height = outs.shape[0]
        feature_width = outs.shape[1]
        preds = []
        confidences = []
        boxes = []
        ret = []
        for h in range(feature_height):
            for w in range(feature_width):
                data = outs[h][w]
                obj_score, cls_score = data[0], data[5:].max()
                score = (obj_score**0.6) * (cls_score**0.4)
                if score > self.confThreshold:
                    classId = np.argmax(data[5:])
                    # 检测框中心点偏移
                    x_offset, y_offset = tanh(data[1]), tanh(data[2])
                    # 检测框归一化后的宽高
                    box_width, box_height = sigmoid(data[3]), sigmoid(data[4])
                    # 检测框归一化后中心点
                    box_cx = (w + x_offset) / feature_width
                    box_cy = (h + y_offset) / feature_height
                    x1, y1 = box_cx - 0.5 * box_width, box_cy - 0.5 * box_height
                    x2, y2 = box_cx + 0.5 * box_width, box_cy + 0.5 * box_height
                    x1, y1, x2, y2 = (
                        int(x1 * frameWidth),
                        int(y1 * frameHeight),
                        int(x2 * frameWidth),
                        int(y2 * frameHeight),
                    )
                    preds.append([x1, y1, x2, y2, score, classId])
                    boxes.append([x1, y1, x2 - x1, y2 - y1])
                    confidences.append(score)
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        for i in indices:
            pred = preds[i]
            score, classId = pred[4], int(pred[5])
            x1, y1, x2, y2 = pred[0], pred[1], pred[2], pred[3]
            center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
            ret.append(((center_x, center_y), self.classes[classId], score))
            if self.drawOutput:
                draw_pred(frame, self.classes[classId], score, x1, y1, x2, y2)
        return ret

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 类型名称, 置信度)
        """
        data = self.__preprocess(frame, (self.inpWidth, self.inpHeight))
        input_name = self.session.get_inputs()[0].name
        feature_map = self.session.run([], {input_name: data})[0][0]
        return self.__postprocess(frame, feature_map)


if __name__ == "__main__":
    # 读取图片
    cam = cv2.VideoCapture(0)
    # 模型输入的宽高
    input_width, input_height = 500, 500
    # 加载模型
    fd = FastestDetOnnx(drawOutput=True)
    # 目标检测
    while True:
        img = cam.read()[1]
        start = time.perf_counter()
        ret = fd.detect(img)
        end = time.perf_counter()
        time_ = (end - start) * 1000.0
        print("forward time:%fms" % time_)
        cv2.imshow("img", img)
        if cv2.waitKey(1) == 27:
            break
