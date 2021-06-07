import cv2
from cv2 import aruco
import numpy as np
from threading import Thread
import typing
import time
from algorithm import Algorithm
from server_socket import *

marker_size = 7
marker_separator = 4

# список Aruco маркеров
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# калибровочная доска
board = aruco.GridBoard_create(markersX=5, markersY=7, markerLength=marker_size, markerSeparation=marker_separator,
                               dictionary=dictionary)

parameters = aruco.DetectorParameters_create()

static_marker_id = 3

# объект, содержащий в себе информацию об id маркера, векторе поворота и векторе переноса
class Marker(typing.NamedTuple):
    id: int
    rvecs: np.ndarray
    tvecs: np.ndarray


class CameraThread(Thread):
    def __init__(self, camera_number, calibrate_marker_coordinate):
        Thread.__init__(self)
        self.camera_counter = camera_number
        self.camera_martix = None
        self.dist_coeff = None
        self.transition_matrix_from_camera_to_world = None
        self.algorithm = Algorithm()
        self.calibrate_marker_coordinate = calibrate_marker_coordinate

    def run(self) -> None:
        self.start_processing_frames()

    def start_processing_frames(self):
        cv2.namedWindow(f'result{self.camera_counter}')

        cap = cv2.VideoCapture(self.camera_counter)
        cap.set(3, 1280)
        cap.set(4, 700)

        while True:
            time.sleep(1)
            flag, img = cap.read()
            if flag:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                self.transition_matrix_from_camera_to_world = None
                self.find_markers_and_markers_coordinates(gray)

            if cv2.waitKey(5) == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    # Метод для нахождения Aruco маркеров и их координат в мировой системе счисления
    def find_markers_and_markers_coordinates(self, gray):
        # corners - 2D вектора углов маркера
        # ids - id обнаруженных маркеров
        self.corners, self.ids, reject = self.detect_markers(gray)

        if self.ids is not None and self.corners is not None:

            # отрисовка найдённых маркеров
            aruco.drawDetectedMarkers(gray, self.corners, self.ids)

            if self.camera_martix is None:
                self.calibrate_camera(np.array([len(self.ids)]), gray.shape[::-1])
            else:
                self.find_rvecs_and_tvecs()

                self.convert_data_to_marker()

                self.find_transition_matrix()

                self.find_world_coordinates_for_all_markers()

    def convert_data_to_marker(self):
        self.static_marker_index = np.where(self.ids == static_marker_id)[0][0] if self.ids.__contains__(static_marker_id) else - 1
        self.markers = [Marker(self.ids[i], self.rvecs[i], self.tvecs[i]) for i in range(len(self.rvecs)) if
                        self.static_marker_index != i]
        self.static_marker = Marker(self.ids[self.static_marker_index], self.rvecs[self.static_marker_index],
                                    self.tvecs[self.static_marker_index]) if self.static_marker_index != -1 else None
    @staticmethod
    # Поиск маркер в кадре
    def detect_markers(gray):
        return aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # Калибровка камеры
    def calibrate_camera(self, counter, shape):
        # distCoeff - вектор коэффициентов искажения
        _, self.camera_martix, self.dist_coeff, _, _ = aruco.calibrateCameraAruco(
            corners=self.corners, ids=self.ids, counter=counter, board=board, imageSize=shape,
            cameraMatrix=None, distCoeffs=None)

    # получаем вектора поворотов и положенией маркеров относительно системы координат камеры
    def find_rvecs_and_tvecs(self):
        self.rvecs, self.tvecs, _ = aruco.estimatePoseSingleMarkers(self.corners, marker_size,
                                                                    self.camera_martix, self.dist_coeff)

    # Находим маркеру переходу из системы координат камеры в мировую систему координат
    def find_transition_matrix(self):
        # базисные единичные вектора для калибровки по статическому маркеру
        x = np.array([1, 0, 0]).reshape(1, 3)
        y = np.array([0, 1, 0]).reshape(1, 3)
        z = np.array([0, 0, 1]).reshape(1, 3)

        # находим матрицу перехода из мировой системы в систему маркера
        transform_matrix = np.linalg.pinv(
            self.concatenate_by_vert_and_horiz(np.vstack((np.vstack((x, y)), z)), self.calibrate_marker_coordinate)
        )

        if self.static_marker is not None:
            # получаем из вектора поворота матрицу поворота
            rvec_mat, _ = cv2.Rodrigues(self.static_marker.rvecs)
            # конкатенируем матрицы и получаем квадратную матрицу для аффинного преобразования
            rotate_matrix = self.concatenate_by_vert_and_horiz(rvec_mat, self.static_marker.tvecs.T).dot(transform_matrix)
            self.transition_matrix_from_camera_to_world = np.linalg.pinv(rotate_matrix)

    @staticmethod
    def concatenate_by_vert_and_horiz(first_matrix, second_matrix):
        return np.vstack((np.hstack((first_matrix, second_matrix)), np.array([0, 0, 0, 1])))

    def find_world_coordinates_for_all_markers(self):
        # Координаты i-го маркера в мировой системе координат
        if self.transition_matrix_from_camera_to_world is not None:
            coordinate = [(marker.id, self.transition_matrix_from_camera_to_world.dot(
                        np.vstack((marker.tvecs.T, np.array([1]))))[0:3, 0]) for marker in self.markers]
            self.algorithm.initialise_coordinate(coordinate)
            connection.send(coordinate)


if __name__ == '__main__':
    first_marker_coordinate = np.array([[0], [0], [50]])
    second_marker_coordinate = np.array([[160], [200], [50]])
    initialise_server_socket()
    thread0 = CameraThread(0, first_marker_coordinate)
    thread1 = CameraThread(1, second_marker_coordinate)
    thread0.start()
    thread1.start()
    thread0.join()
    thread1.join()
