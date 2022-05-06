import numpy as np
import math

dt = 0.001 / 4


class Slalom:
    base_time = 0
    v = 0
    rad = 0
    ang = 0
    Et = 0
    limit_time_count = 0
    base_alpha = 0
    pow_n = 0
    start_theta = 0
    res = {}
    end_pos = {}
    start_offset = 0
    end_offset = 0
    base_ang = 0
    type = ""
    start_offset_list = []
    end_offset_list = []
    turn_offset = {"x": 0, "y": 0}
    cell_size = 90
    half_cell_size = 45

    def __init__(self, v, rad, n, ang, end_pos, type):
        self.v = v
        self.rad = rad
        self.ang = ang * math.pi / 180
        self.base_ang = ang
        self.pow_n = n
        self.end_pos = end_pos
        self.type = type
        if n == 2:
            self.Et = 0.603450161218938087668
        elif n == 4:
            self.Et = 0.763214618198974433973
        self.base_alpha = v / rad

    def calc_base_time(self):
        c = 0
        t1 = 0
        tmp_dt = 0.001 / 64

        while c < 10000000:
            t1 = t1 + tmp_dt
            if (2 * self.v / self.rad * self.Et * t1) >= self.ang:
                self.base_time = t1
                self.limit_time_count = t1 * 2 / dt
                return
            c = c + 1

    def calc(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
        res["alpha"] = np.array([0])
        res["w"] = np.array([0])
        tmp_w = 0
        tmp_theta = start_ang * math.pi / 180
        tmp_x = 0
        tmp_y = 0
        for i in range(1, int(self.limit_time_count + 1)):
            tmp_alpha = self.base_alpha * self.calc_neipire(dt * i, self.base_time, self.pow_n)
            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt
            tmp_x = tmp_x + self.v * math.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + self.v * math.sin(self.start_theta + tmp_theta) * dt
            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)
        print(np.max(res["w"])**2*self.rad/9.81/1000)
        self.res = res
        return res

    def calc_neipire(self, t, s, N):
        z = 1
        t = t / s
        P = math.pow((t - z), N - z)
        Q = P * (t - z)
        res = -N * P / ((Q - z) * (Q - z)) * math.pow(math.exp(1), z + z / (Q - z)) / s
        return res

    def calc_offset_dist(self):
        a = math.sin(self.ang)
        b = math.cos(self.ang)
        if self.ang == 0:
            a = 1
            b = 0
        self.end_offset = (self.end_pos["y"] - self.res["y"][-1]) / a
        self.start_offset = (self.end_pos["x"] - self.res["x"][-1]) - self.end_offset * b
        if self.type == "normal":
            self.start_offset_list = [[self.half_cell_size, self.start_offset + self.half_cell_size], [0, 0]]
            self.end_offset_list = [[self.res["x"][-1] + self.start_offset + self.half_cell_size,
                                     self.res["x"][-1] + self.start_offset + self.half_cell_size],
                                    [self.res["y"][-1], self.res["y"][-1] + self.end_offset]]
        elif self.type == "large":
            self.start_offset_list = [[0, self.start_offset], [0, 0]]
            self.end_offset_list = [[self.res["x"][-1] + self.start_offset, self.res["x"][-1] + self.start_offset],
                                    [self.res["y"][-1], self.res["y"][-1] + self.end_offset]]
        elif self.type == "dia45":
            self.start_offset_list = [[0, self.start_offset], [0, 0]]
            self.end_offset_list = [[self.res["x"][-1] + self.start_offset,
                                     self.res["x"][-1] + self.start_offset + self.end_offset / math.sqrt(2)],
                                    [self.res["y"][-1], self.res["y"][-1] + self.end_offset / math.sqrt(2)]]
        elif self.type == "dia135":
            self.start_offset_list = [[0, self.start_offset], [0, 0]]
            self.end_offset_list = [[self.res["x"][-1] + self.start_offset,
                                     self.res["x"][-1] + self.start_offset - self.end_offset / math.sqrt(2)],
                                    [self.res["y"][-1], self.res["y"][-1] + self.end_offset / math.sqrt(2)]]
        elif self.type == "dia45_2":
            self.start_offset = (self.half_cell_size - self.res["x"][-1]) / math.sin(math.pi / 4)
            self.end_offset = self.cell_size - self.res["y"][-1] - self.start_offset * math.sin(math.pi / 4)
            self.start_offset_list = [
                [self.half_cell_size, self.start_offset * math.sin(math.pi / 4) + self.half_cell_size]
                , [0, self.start_offset * math.sin(math.pi / 4)]]
            self.end_offset_list = [
                [self.res["x"][-1] + self.start_offset * math.sin(math.pi / 4) + self.half_cell_size,
                 self.res["x"][-1] + self.start_offset * math.sin(math.pi / 4) + self.half_cell_size],
                [self.res["y"][-1] + self.start_offset * math.sin(math.pi / 4),
                 self.res["y"][-1] + self.start_offset * math.sin(math.pi / 4) + self.end_offset]]
        elif self.type == "dia135_2":
            self.start_offset = (self.cell_size - self.res["y"][-1]) / math.sin(math.pi / 4)
            self.end_offset = math.fabs(
                -self.half_cell_size - self.res["x"][-1] - self.start_offset * math.sin(math.pi / 4))

            self.start_offset_list = [
                [self.half_cell_size, self.start_offset * math.sin(math.pi / 4) + self.half_cell_size],
                [0, self.start_offset * math.sin(math.pi / 4)]]
            self.end_offset_list = [
                [self.res["x"][-1] + self.start_offset * math.sin(math.pi / 4) + self.half_cell_size,
                 self.res["x"][-1] + self.start_offset * math.sin(math.pi / 4) - self.end_offset + self.half_cell_size],
                [self.res["y"][-1] + self.start_offset * math.sin(math.pi / 4),
                 self.res["y"][-1] + self.start_offset * math.sin(math.pi / 4)]]

        elif self.type == "dia90":
            self.half_cell_size = 0
            self.end_offset = (self.cell_size / math.sqrt(2) - self.res["y"][-1]) / a
            self.start_offset = (self.cell_size / math.sqrt(2) - self.res["x"][-1]) - self.end_offset * b

            self.start_offset_list = [[self.half_cell_size, self.start_offset + self.half_cell_size], [0, 0]]
            self.end_offset_list = [[self.res["x"][-1] + self.start_offset + self.half_cell_size,
                                     self.res["x"][-1] + self.start_offset + self.half_cell_size],
                                    [self.res["y"][-1], self.res["y"][-1] + self.end_offset]]

        self.turn_offset["x"] = self.start_offset_list[0][1]
        self.turn_offset["y"] = self.start_offset_list[1][1]
        # print(self.start_offset, self.end_offset)
