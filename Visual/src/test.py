from Vision_Net import FastestDet
import cv2
import time
camera =cv2.VideoCapture(0)

deep = FastestDet(drawOutput=True)
t1 = time.time()
cnt = 0
k2 = 0
while True:
    cnt += 1
    img = camera.read()[1]
    if time.time() - t1 != 0:
        cv2.putText(img, "FPS {0}".format(float('%.1f' % (cnt / (time.time() - t1)))), (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                        2)

    if img is None:
        continue
    get = deep.detect(img)
    if get != []:
        print(get)
    cv2.imshow("Result",img)
    if cv2.waitKey(10) == 27:
        break
cv2.destroyAllWindows()
