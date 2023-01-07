import cv2
import numpy as np
import os
import time
#import serial
WIDTH = 640
HEIGHT = 480

DIRECTION = 'R'

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
#capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

COLOR_DICT = {
                'highred':[np.array([156,43,46]),np.array([180,255,255])],
                'lowred':[np.array([0,43,46]),np.array([10,255,255])],
                'green':[np.array([35,43,46]),np.array([77,255,255])],
                'blue':[np.array([100,43,46]),np.array([124,255,255])],
                'yellow':[np.array([26,43,46]),np.array([34,255,255])],
                'orange':[np.array([11,43,46]),np.array([25,255,255])]}
COLORS = ['highred','lowred','green','blue','yellow','orange']

def nms(dets, nmsThreshold):
    """
    非极大值抑制
    dets: [[x1, y1, x2, y2, score], [x1, y1, x2, y2, score], ...]
    """
    # dets:N*M,N是bbox的个数，M的前4位是对应的（x1,y1,x2,y2），第5位是对应的分数
    # #thresh:0.3,0.5....
    if dets.shape[0] == 0:
        return []
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]
    scores = dets[:, 4]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # 求每个bbox的面积
    order = scores.argsort()[::-1]  # 对分数进行倒排序
    keep = []  # 用来保存最后留下来的bboxx下标
    while order.size > 0:
        i = order[0]  # 无条件保留每次迭代中置信度最高的bbox
        keep.append(i)
        # 计算置信度最高的bbox和其他剩下bbox之间的交叉区域
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        # 求交叉区域的面积占两者（置信度高的bbox和其他bbox）面积和的比例
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        # 保留ovr小于thresh的bbox，进入下一次迭代。
        inds = np.where(ovr <= nmsThreshold)[0]
        # 因为ovr中的索引不包括order[0]所以要向后移动一位
        order = order[inds + 1]
    output = []
    for i in keep:
        output.append(dets[i].tolist())
    return output


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


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
class FastestDet:
    def __init__(self, confThreshold=0.5, nmsThreshold=0.4, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "C:/Users/4869/Desktop/Car_project/SolutionsNew/train10/FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "C:/Users/4869/Desktop/Car_project/SolutionsNew/train10/FastestDet.onnx")
        self.classes = list(map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 150
        self.inpHeight = 150
        self.net = cv2.dnn.readNet(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.drawOutput = drawOutput

    def post_process(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        outs = outs.transpose(1, 2, 0)  # 将维度调整为 (H, W, C)
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
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.inpWidth, self.inpHeight))
        self.net.setInput(blob)
        pred = self.net.forward(self.net.getUnconnectedOutLayersNames())[0][0]
        return self.post_process(frame, pred)
#识别交通灯颜色    
class ColorRecognize():
    def __init__(self):
        #颜色区间
        self.color_dict = COLOR_DICT
        self.colors = COLORS
        self.color = None
        self.cropy0 = 200  
        self.cropy1 = 300
        self.cropx0 = 200
        self.cropx1 = 400
    #记录白色像素个数
    def get_white_pixel(self,img):
        return len(np.where(img == 255)[0])

    #通过计算哪种颜色的掩膜的白色像素数量最多确定颜色
    def get_color(self,img):
        try:
            img = img[self.cropy0:self.cropy1,self.cropx0:self.cropx1]  #缩小区域
            img_test = img.copy()   #在缩小后的区域找最亮区域
            img_test = cv2.cvtColor(img_test, cv2.COLOR_BGR2GRAY)
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(img_test)

            img_test2 = img.copy() #这里是为了显示出第一次缩小的图像，在图像中圈出最亮区域
            #在上次缩小的区域中截取出最亮点的区域作为目标区域
            img = img[maxLoc[1]-20:maxLoc[1]+20,maxLoc[0]-20:maxLoc[0]+20]

            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            #高斯模糊
            hsv = cv2.GaussianBlur(hsv,(5,5),0)
            maxcount = 0
            color = None
            for i in self.colors:
                mask = cv2.inRange(hsv,self.color_dict[i][0],self.color_dict[i][1])
                kernel = np.ones((3,3),np.uint8)
                #填补孔洞
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,iterations=5)
                res = cv2.bitwise_and(img,img,mask=mask)
                
                count = self.get_white_pixel(mask)
                if count > maxcount:
                    maxcount = count
                    if i == "highred" or i == "lowred":
                        i = "red"
                    color = i

            cv2.circle(img_test2, maxLoc, 20, (255, 0, 0), 2)
            #cv2.imshow('2',mask)
            cv2.imshow('1',img)
            cv2.imshow('3',img_test2)

            return color
        except Exception:
            pass
    def start(self,frame):
        print(self.get_color(frame))

#############################################################################################
#巡线
class Findroute():
   
    def __init__(self,direct):
        #self.RouteRegionParams = [HEIGHT//2+130,HEIGHT//2+150,WIDTH//12+20,WIDTH-WIDTH//12-20]
        self.RouteRegionParams = [HEIGHT//2+130,HEIGHT//2+150,WIDTH//12+30,WIDTH-WIDTH//12-10]
        self.UpWaitstopRegionParams = [HEIGHT//2+60,HEIGHT-160,WIDTH//3-50,WIDTH*2//3+50]
        self.DownWaitstopRegionParams = [HEIGHT//2,HEIGHT//2+60,WIDTH//3-50,WIDTH*2//3+50]
        self.threshold = 130
        self.direction = direct
    def start(self,img):
        try:
            direction = self.direction
            #转灰度图
            Gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            [cropx1,cropx2,cropy1,cropy2] = self.RouteRegionParams   #裁剪的ROI参数
        
            Routeregion = Gray[cropx1:cropx2,cropy1:cropy2]    #ROI
            
            Routeregion = cv2.threshold(Routeregion, self.threshold, 255, cv2.THRESH_BINARY)[1]    #长方形条
            Gray = cv2.threshold(Gray, self.threshold, 255, cv2.THRESH_BINARY_INV)[1]    #原始灰度图

            #预处理ROI
            Routeregion = cv2.GaussianBlur(Routeregion,(5,5),0)
            kernel = np.ones((3,3),np.uint8)
            Routeregion = cv2.morphologyEx(Routeregion, cv2.MORPH_OPEN, kernel,iterations=1)
            Routeregion = cv2.morphologyEx(Routeregion, cv2.MORPH_CLOSE, kernel,iterations=1)
            
            #寻找ROI中黑线轮廓
            contours = cv2.findContours(Routeregion, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
            delta = np.array([[cropy1,cropx1]]) #变换坐标的参数(裁切后的图的坐标转化为原灰度图坐标)
            print(self.direction)
            if self.direction == "R":
                contour = contours[1]   #与左右转弯有关，这是参考的轮廓
            elif self.direction == "L":
                contour = contours[-1] 
            contour += delta    #变换轮廓坐标
            x,y,w,h = cv2.boundingRect(contour)  #拟合为矩形以找出轮廓边界
                
            #根据轮廓边界确定线的中心点
            center1 = (x+w+10,y+h//2)
            center2 = (WIDTH//2,HEIGHT//2)

            #画出点
            cv2.drawContours(img, contour, -1, (255,0,0), 3)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),3)
            cv2.circle(img,center1,0,(0,0,255),10)
            cv2.circle(img,center2,0,(255,0,0),10)

            #图像中心点与目标点偏移距离
            direction = center1[0] - center2[0]
            if direction > 0:
                if direction >= 150:
                    direction = 150
                #ser.write('#{}#\r\n'.format(int(direction)).encode('utf-8'))
                #print("右转")
            elif direction < 0:
                if direction <= -150:
                    direction = -150
                #ser.write('#{}#\r\n'.format(int(direction)).encode('utf-8'))
                #print('左转')
            #print('juli',direction)
            
            get = deep.detect(img)[0][1]
            if get == 'waitstop':
                #ser.write('w\r\n'.encode('utf-8'))
                print("等停")
            elif get == 'stop':
                #ser.write('s\r\n'.encode('utf-8'))
                print("停止")
            elif get == 'L':
                direction = 'L'
            elif get == 'R':
                direction = 'R'
            self.direction = direction
            
        except Exception:
            pass
        cv2.rectangle(img,(cropy1,cropx1),(cropy2,cropx2),(0,255,0),3)
        cv2.imshow('1',img) 
#############################################################################################

deep = FastestDet(drawOutput=True)
#B = ColorRecognize()
C = Findroute(direct='R')
while True:
    ret, frame = capture.read()
    C.start(frame)
    if cv2.waitKey(10) == 27:
        break
cv2.destroyAllWindows() 


    




