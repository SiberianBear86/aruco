import cv2
from cv2 import aruco
import numpy as np
from threading import Thread


class CameraThread(Thread):

    def __init__(self, camera_number):
        Thread.__init__(self)
        self.camera_counter = camera_number

    def run(self) -> None:
        calibrate_camera(self.camera_counter)


def calibrate_camera(camera_counter):
    cv2.namedWindow(f'result{camera_counter}')

    # генерируется список маркеров
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    #dictionary = aruco.Dictionary_get(aruco.DICT_4X4_250)

    ## доска нужна для калибровки
    board = aruco.GridBoard_create(
        markersX=5,
        markersY=7,
        markerLength=0.04,
        markerSeparation=0.01,
        dictionary=dictionary)

    boardImg = board.draw((1000, 1000))

    cv2.imwrite("boardImg.png", boardImg)

    cap = cv2.VideoCapture(camera_counter)
    cap.set(3, 1280)
    cap.set(4, 700)
    # #мой кастомный маркер
    # my_marker = aruco.custom_dictionary(11, 4)
    # my_marker_img = aruco.drawMarker(my_marker, 3, 35)
    # cv2.imwrite("my_marker_img.png", my_marker_img)

    while True:
        flag, img = cap.read()
        if flag:
            img_copy = img.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            parameters = aruco.DetectorParameters_create()

            # corners - 2D вектора углов маркера
            # ids - id обнаруженных маркеров
            corners, ids, reject = aruco.detectMarkers(gray, dictionary, parameters=parameters)

            #отрисовка найдённых маркеров
            aruco.drawDetectedMarkers(img_copy, corners, ids)

            if ids is not None and corners is not None and len(ids) == len(board.ids):
                X = np.array([1, 0, 0]).reshape(1, 3)
                Y = np.array([0, 1, 0]).reshape(1, 3)
                Z = np.array([0, 0, 1]).reshape(1, 3)

                center_of_marker = np.array([[3], [3], [0]])

                # находим матрицу перехода из мировой системы в систему маркера
                transform_matrix = np.linalg.inv(np.vstack((np.hstack((np.vstack((np.vstack((X, Y)), Z)), center_of_marker)), np.array([0, 0, 0, 1]))))

                inverse_affine_matrices = []
                counter, corners_list, id_list = [], corners, ids

                corners_list = np.vstack((corners_list, corners))
                id_list = np.vstack((id_list, ids))

                counter.append(len(ids))
                counter = np.array(counter)

                # Калибровка камеры
                # distCoeff - вектор коэффициентов искажения
                flag, cameraMatrix, distCoeff, _, _ = aruco.calibrateCameraAruco(
                    corners=corners_list,
                    ids=id_list,
                    counter=counter,
                    board=board,
                    imageSize=gray.shape,
                    cameraMatrix=None,
                    distCoeffs=None)

                # получаем вектора поворотов и положенией маркеров относительно системы координат камеры
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeff)

                # списки матриц поворотов и векторов переноса
                inverse_rvecs, inverse_tvecs = [], []

                for i in range(len(rvecs)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    #Функция, которая преобразует вектор поворота в матрицу поворота, и обратно
                    rvec_mat, _ = cv2.Rodrigues(rvec)
                    #конкатенируем матрицы и получаем квадратную матрицу для аффинного преобразования
                    rotate_matrix = (np.vstack((np.hstack((rvec_mat, tvec.T)), np.array([0, 0, 0, 1])))).dot(transform_matrix)
                    #получаем обратную матрицу
                    inv_rotate_matrix = np.linalg.inv(rotate_matrix)

                    #берём срез матрицы
                    rvec_mat = inv_rotate_matrix[0:3, 0:3]
                    tvec = inv_rotate_matrix[0:3, 3]

                    #преобразуем матрицу поворота в вектор поворота
                    rvec, _ = cv2.Rodrigues(rvec_mat)

                    inverse_rvecs.append(rvec)
                    inverse_tvecs.append(tvec.T)

                # Берём среднее арифметическое по всем обратным векторам поворота
                r_average = np.average(np.array(inverse_rvecs), axis=0)
                # Берём среднее арифметическое по всем обратным векторам положения
                t_average = np.average(np.array(inverse_tvecs), axis=0)
                # получаем матрицу поворота на основе усредненного вектора поворота
                r_mat_average, _ = cv2.Rodrigues(r_average)
                # получаем обратну усреднённую матрицу на основе усреднённой матрицы поворота и усреднённого вектора переноса
                inverse_rotate_matrix_average = np.hstack((r_mat_average, np.array([t_average]).T))

                # Координаты i-го маркера в мировой системе координат
                world_coordinates = inverse_rotate_matrix_average.dot(np.vstack((tvecs[1].T, np.array([1]))))[0:3, 3]

            cv2.imshow(f'result{camera_counter}', img_copy)

            if cv2.waitKey(5) == 27:
                break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    thread0 = CameraThread(0)
    #thread1 = CameraThread(1)
    thread0.start()
    #thread1.start()
    thread0.join()
    #thread1.join()