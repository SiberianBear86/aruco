import numpy as np
import math

robot_length = 8

id_current_robot = 3

table_length = 300
table_width = 200


class Algorithm:
    def __init__(self):
        self.previous_coordinate = None
        self.current_coordinate = None
        self.previous_enemy_coordinate = None
        self.current_enemy_coordinate = None
        self.current_speed = 0
        self.enemy_speed = 0
        self.current_line = {}
        self.enemy_line = {}

    def initialise_coordinate(self, coordinates):
        for coordinate in coordinates:
            if self.previous_coordinate is None and self.previous_enemy_coordinate is None:
                if coordinate[0] == id_current_robot:
                    self.current_coordinate = coordinate[1]
                else:
                    self.current_enemy_coordinate = coordinate[1]
            else:
                if coordinate[0] == id_current_robot:
                    self.previous_coordinate = self.current_coordinate
                    self.current_coordinate = coordinate[1]
                else:
                    self.previous_enemy_coordinate = self.current_enemy_coordinate
                    self.current_enemy_coordinate = coordinate[1]
        if self.previous_coordinate is not None and self.previous_enemy_coordinate is not None:
              self.algorithm_for_search_intersections_of_robots()


    # метод определяет столкновение роботов, в случаи если их траектории маршрута паралельны или робот противника стоит на месте
    def find_intersection_in_parallel_lines_case(self):
        # прямые паралельны, но нам нужно понять что они не столкнутся по габаритам
        # находим расстояние между координатами, чтобы определить направления движения роботов
        distance_between_current_coordinate_and_current_enemy_coordinate = math.sqrt((self.x2 - self.x4) ** 2 + (self.y2 - self.y4) ** 2)
        distance_between_current_coordinate_and_previous_enemy_coordinate = math.sqrt((self.x2 - self.x3) ** 2 + (self.y2 - self.y3) ** 2)
        distance_between_previous_coordinate_and_current_enemy_coordinate = math.sqrt((self.x1 - self.x4) ** 2 + (self.y1 - self.y4) ** 2)

        # положим 2 точки на 1 прямую, для подсчёта времени
        y_enemy_coordinate_on_current_line = self.y4 - self.enemy_line["b"] + self.current_line["b"]

        distance_between_robots = math.sqrt((self.x2 - self.x4) ** 2 + (self.y2 - y_enemy_coordinate_on_current_line) ** 2) - robot_length

        time_to_intersection = distance_between_robots / (self.enemy_speed + self.current_speed)

        if distance_between_current_coordinate_and_current_enemy_coordinate <= distance_between_current_coordinate_and_previous_enemy_coordinate and \
                distance_between_previous_coordinate_and_current_enemy_coordinate > distance_between_current_coordinate_and_current_enemy_coordinate and \
                abs(self.current_line["b"] - self.enemy_line["b"]) < robot_length / 2 and time_to_intersection < 2:
            print("Стоп!")

    # метод позволяет найти точку пересечения траекторий маршурта роботов
    def find_intersection_point(self):
        # найти точку пересечения
        c = -1 * (self.current_line["b"] - self.enemy_line["b"])
        if self.x < 0:
            self.x *= -1
            c *= -1
        self.x /= c
        self.y = self.current_line["a"] * self.x + self.current_line["b"]
        print(f"Точка пересечения O ({self.x}, {self.y})")

    # метод находит пересечение двух линий
    def find_intersection_of_robots(self):
        # находим расстояние между координатами, чтобы определить направления движения роботов
        distance_between_intersection_point_and_previous_coordinate = math.sqrt((self.x1 - self.x) ** 2 + (self.y1 - self.y) ** 2)
        distance_between_intersection_point_and_current_coordinate = math.sqrt((self.x2 - self.x) ** 2 + (self.y2 - self.y) ** 2)
        distance_between_intersection_point_and_previous_enemy_coordinate = math.sqrt((self.x3 - self.x) ** 2 + (self.y3 - self.y) ** 2)
        distance_between_intersection_point_and_current_enemy_coordinate = math.sqrt((self.x4 - self.x) ** 2 + (self.y4 - self.y) ** 2)

        if distance_between_intersection_point_and_current_coordinate < distance_between_intersection_point_and_previous_coordinate and \
                distance_between_intersection_point_and_current_enemy_coordinate < distance_between_intersection_point_and_previous_enemy_coordinate \
                and 0 < self.x < table_length and 0 < self.y < table_width:  # точка пересечения лежит внутри игрового стола
            # находим время, которое нужно каждому роботу до точки пересечения
            first_time = distance_between_intersection_point_and_current_coordinate / self.current_speed
            second_time = distance_between_intersection_point_and_current_enemy_coordinate / self.enemy_speed

            if abs(first_time - second_time) < 3:
                print("Стоп!")

    # Алгоритм который просчитывает возможные пересечения траекторий маршрутов роботов
    # рассматривются случаи
    # 1) Линии траекторий роботов пересекаются в рамках стола и ТП лежит не вблизи краёв стола
    # 2) ЛТ роботов паралельны и по габаритам роботы не пересекаются
    # 3) Робот-соперник стоит на месте
    # Учитывается скорость роботов, высчитывается время до столкновения
    def algorithm_for_search_intersections_of_robots(self):
        self.x1, self.y1, _ = self.previous_coordinate
        self.x2, self.y2, _ = self.current_coordinate
        self.x3, self.y3, _ = self.previous_enemy_coordinate
        self.x4, self.y4, _ = self.current_enemy_coordinate

        self.current_speed = math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)
        self.enemy_speed = math.sqrt((self.x3 - self.x4) ** 2 + (self.y3 - self.y4) ** 2)

        if self.current_speed != 0:
            if self.x1 == self.x2: # случай, когда робот движется по прямой вида y = b
                self.current_line = {"a": 0, "b": self.y1}
            else:
                self.current_line = {"a": (self.y2 - self.y1) / (self.x2 - self.x1), "b": -1 * self.x1 * (self.y2 - self.y1) / (self.x2 - self.x1) + self.y1}

            if self.x3 == self.x4: # случай, когда робот движется по прямой вида y = b
                self.enemy_line = {"a": 0, "b": self.y3}
            else:
                self.enemy_line = {"a": (self.y4 - self.y3) / (self.x4 - self.x3), "b": -1 * self.x3 * (self.y4 - self.y3) / (self.x4 - self.x3) + self.y3}

            self.x = self.current_line["a"] - self.enemy_line["a"]

            if self.x == 0 or self.enemy_speed == 0:
                self.find_intersection_in_parallel_lines_case()
            else:
                self.find_intersection_point()
                self.find_intersection_of_robots()