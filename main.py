import cv2
from cv2 import aruco
import numpy as np

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


def nothing(*arg):
    pass

def my_video():
    cv2.namedWindow("result")
    # cv2.namedWindow("settings")  # создаем окно настроек
    #
    # cv2.createTrackbar('h1', 'settings', 0, 255, nothing)
    # cv2.createTrackbar('s1', 'settings', 0, 255, nothing)
    # cv2.createTrackbar('v1', 'settings', 0, 255, nothing)
    # cv2.createTrackbar('h2', 'settings', 255, 255, nothing)
    # cv2.createTrackbar('s2', 'settings', 255, 255, nothing)
    # cv2.createTrackbar('v2', 'settings', 255, 255, nothing)
    # crange = [0, 0, 0, 0, 0, 0]

    # генерируется список маркеров
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    ## доска нужна для калибровки
    board = aruco.GridBoard_create(
        markersX=5,
        markersY=7,
        markerLength=0.04,
        markerSeparation=0.01,
        dictionary=aruco_dict)

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 700)
    #
    hsv_min = np.array((65, 21, 23), np.uint8)
    hsv_max = np.array((142, 88, 55), np.uint8)
    id = np.array([1])
    color_yellow = (0, 255, 255)
    myKok = aruco.custom_dictionary(11, 4)
    lalka = aruco.drawMarker(myKok, 3, 35)
    cv2.imwrite("lol.png", lalka)

    while True:
        flag, img = cap.read()
        img_copy = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # считываем значения бегунков
        h1 = cv2.getTrackbarPos('h1', 'settings')
        s1 = cv2.getTrackbarPos('s1', 'settings')
        v1 = cv2.getTrackbarPos('v1', 'settings')
        h2 = cv2.getTrackbarPos('h2', 'settings')
        s2 = cv2.getTrackbarPos('s2', 'settings')
        v2 = cv2.getTrackbarPos('v2', 'settings')
        #
        # формируем начальный и конечный цвет фильтра
        # hsv_min = np.array((h1, s1, v1), np.uint8)
        # hsv_max = np.array((h2, s2, v2), np.uint8)

        # img = cv2.flip(img, 1)  # отражение кадра вдоль оси Y
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # применяем цветовой фильтр
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)

        # dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        # dictionary = aruco.custom_dictionary(11, 4)
        # lalka = aruco.drawMarker(dictionary, 0, 35)
        # cv2.imwrite("lol.png", lalka)

        corners, ids, reject = aruco.detectMarkers(img, myKok)

        aruco.drawDetectedMarkers(img_copy, corners, ids)

        ret, matrix, distCoeff, _, _ = cv2.calibrateCamera(
            objectPoints=board.objPoints,
            imagePoints=corners,
            imageSize=gray.shape,  # [::-1], # may instead want to use gray.size
            cameraMatrix=None,
            distCoeffs=None
        )

        # ищем контуры и складируем их в переменную contours
        # contours0, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #
        # # отображаем контуры поверх изображения
        # cv2.drawContours(img, contours0, -1, (255, 0, 0), 3, cv2.LINE_AA, hierarchy, 1)
        # for cnt in contours0:
        #     rect = cv2.minAreaRect(cnt)  # пытаемся вписать прямоугольник
        #     box = cv2.boxPoints(rect)  # поиск четырех вершин прямоугольника
        #     box = np.int0(box)  # округление координат
        #     cv2.drawContours(img, [box], 0, (255, 0, 0), 2)  # рисуем прямоугольник

        # moments = cv2.moments(thresh, 1)
        # dM01 = moments['m01']
        # dM10 = moments['m10']
        # dArea = moments['m00']
        # if dArea > 100:
        #     x = int(dM10 / dArea)
        #     y = int(dM01 / dArea)
        #     cv2.circle(img, (x, y), 5, color_yellow, 2)
        #     cv2.putText(img, "%d-%d" % (x, y), (x + 10, y - 10),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, color_yellow, 2)
        cv2.imshow("result", img_copy)#img)

        if cv2.waitKey(5) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')
    for i in dir(aruco):
        print(i + "\n")
    my_video()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
