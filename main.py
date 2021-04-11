import cv2
from cv2 import aruco
import numpy as np
from functools import reduce


def calibrate_camera():
    cv2.namedWindow("result")

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

    cap = cv2.VideoCapture(0)
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
                inverse_athenian_matrices = []
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

                for i in range(len(rvecs)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    #Функция, которая преобразует вектор поворота в матрицу поворота, и обратно
                    rvec_mat, _ = cv2.Rodrigues(rvec)
                    #конкатенируем матрицы и получаем квадратную матрицу для аффинного преобразования
                    rotate_matrix = np.vstack((np.hstack((rvec_mat, tvec.T)), np.array([0, 0, 0, 1])))
                    #получаем обратную матрицу
                    inv_rotate_matrix = np.linalg.inv(rotate_matrix)
                    # собираем все матрицы для дальнейшего более точного подсчёта обратной матрицы
                    inverse_athenian_matrices.append(inv_rotate_matrix)

                inverse_rvecs, inverse_tvecs = [], []

                for matrix in inverse_athenian_matrices:
                    rvec_mat = matrix[0:3, 0:3]
                    tvec = matrix[0:3, 3]
                    rvec, _ = cv2.Rodrigues(rvec_mat)
                    inverse_rvecs.append(rvec)
                    inverse_tvecs.append(tvec.T)

                # Берём среднее арифметическое по всем обратным векторам поворота
                r_average = np.average(np.array(inverse_rvecs), axis=0)
                # Берём среднее арифметическое по всем обратным векторам положения
                t_average = np.average(np.array(inverse_tvecs), axis=0)
                r_mat_average, _ = cv2.Rodrigues(r_average)
                inverse_rotate_matrix_average = np.vstack((np.hstack((r_mat_average, np.array([t_average]).T)), np.array([0, 0, 0, 1])))

            cv2.imshow("result", img_copy)

            if cv2.waitKey(5) == 27:
                break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    calibrate_camera()