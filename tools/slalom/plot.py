import numpy as np
import matplotlib.pyplot as plt
import math

from slalom import Slalom


class Plot:
    def exe(self, type, offset):

        # fig = plt.figure(figsize=(5, 5), dpi=500)
        fig = plt.figure(dpi=100)
        trj = fig.add_subplot(111)
        trj.set(facecolor="dimgrey")
        # trj.set(facecolor="black")
        v = 300
        rad = 54
        n = 2
        tgt_ang = 90
        slip_gain = 250
        start_pos = {"x": 0, "y": 0}
        end_pos = {"x": 90, "y": 90}
        start_ang = 0
        if type == "normal":
            rad = 26
            n = 2
            tgt_ang = 90
            end_pos = {"x": 45, "y": 45}
            start_ang = 0
        elif type == "large":
            rad = 60
            n = 4
            tgt_ang = 90
            end_pos = {"x": 90, "y": 90}
            start_ang = 0
        elif type == "orval":
            rad = 54
            n = 2
            tgt_ang = 180
            end_pos = {"x": 0, "y": 180}
            start_ang = 0
        elif type == "dia45":
            rad = 50
            n = 4
            tgt_ang = 45
            end_pos = {"x": 90, "y": 45}
            start_ang = 0
        elif type == "dia135":
            rad = 39
            n = 4
            tgt_ang = 135
            end_pos = {"x": 45, "y": 90}
            start_ang = 0
        elif type == "dia45_2":
            rad = 54
            n = 2
            tgt_ang = 45
            end_pos = {"x": 90, "y": 45}
            start_ang = 45
        elif type == "dia135_2":
            rad = 36
            n = 4
            tgt_ang = 135
            end_pos = {"x": -45, "y": 90}
            start_ang = 45
        elif type == "dia90":
            rad = 35
            n = 2
            tgt_ang = 90
            end_pos = {"x": 0, "y": 90}
            start_ang = 0

        sla = Slalom(v, rad, n, tgt_ang, end_pos, slip_gain, type)
        sla.calc_base_time()
        res = sla.calc(start_ang)
        # sla.calc_offset_front()
        start_pos_x = [0, 0]
        start_pos_y = [0, 0]
        sla.calc_offset_dist(start_pos_x, start_pos_y)
        range = [-1000, 1000]

        wall_color = "red"
        wall_width = 6
        wall_alpha = 0.25
        sub_line_color = "silver"
        subline_width = 0.75
        subline_alpha = 0.5
        trj_width = 5
        trj_alpha = 1

        # 壁境界
        trj.plot(range, [45, 45], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot(range, [-45, -45], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([-45, -45], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([45, 45], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot(range, [135, 135], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([135, 135], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)

        # 点線
        trj.plot(range, [0, 0], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot(range, [90, 90], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot(range, [-90, -90], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot([0, 0], range, ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot([90, 90], range, ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)

        # # 前距離
        # trj.plot(sla.start_offset_list[0], sla.start_offset_list[1],
        #          ls="-", color="coral", lw=trj_width, alpha=trj_alpha)
        # # メイン
        # trj.plot(res["x"] + sla.turn_offset["x"], res["y"] + + sla.turn_offset["y"], color="yellow", lw=trj_width,
        #          alpha=trj_alpha)
        # # 後距離
        # trj.plot(sla.end_offset_list[0], sla.end_offset_list[1],
        #          ls="-", color="coral", lw=trj_width, alpha=trj_alpha)

        # trj.plot(sla.start_offset_list[0], sla.start_offset_list[1],
        #          ls="--", color="cyan", lw=1, alpha=trj_alpha)
        # trj.plot(res["x"] + sla.turn_offset["x"], res["y"] + + sla.turn_offset["y"], color="blue", lw=1,
        #          alpha=trj_alpha, ls="--")
        # trj.plot(sla.end_offset_list[0], sla.end_offset_list[1],
        #          ls="--", color="cyan", lw=1, alpha=trj_alpha)

        print('{}:'.format(type))
        print('  v: {}'.format(sla.v))
        print('  ang: {}'.format(sla.base_ang))
        print('  rad: {}'.format(sla.rad))
        print('  pow_n: {}'.format(sla.pow_n))
        print('  time: {}'.format(sla.base_time))
        print('  front: {{ left: {}, right: {} }}'.format(
            sla.start_offset, sla.start_offset))
        print('  back: {{ left: {}, right: {} }}'.format(
            sla.end_offset+offset, sla.end_offset+offset))
        first = [sla.start_offset,  sla.end_offset]
        start_pos_x = [0, 0]
        start_pos_y = [-10, -10]
        sla.calc_offset_dist(start_pos_x, start_pos_y)

        # res = sla.calc_slip(start_ang)
        # sla.calc_offset_dist()
        trj.plot(sla.start_offset_list[0], sla.start_offset_list[1],
                 ls="--", color="cyan", lw=1, alpha=trj_alpha)
        trj.plot(res["x"] + sla.turn_offset["x"], res["y"] + + sla.turn_offset["y"], color="blue", lw=1,
                 alpha=trj_alpha, ls="--")
        trj.plot(sla.end_offset_list[0], sla.end_offset_list[1],
                 ls="--", color="cyan", lw=1, alpha=trj_alpha)

        print('{}:'.format(type))
        print('  v: {}'.format(sla.v))
        print('  ang: {}'.format(sla.base_ang))
        print('  rad: {}'.format(sla.rad))
        print('  pow_n: {}'.format(sla.pow_n))
        print('  time: {}'.format(sla.base_time))
        print('  front: {{ left: {}, right: {} }}'.format(
            sla.start_offset, sla.start_offset))
        print('  back: {{ left: {}, right: {} }}'.format(
            sla.end_offset+offset, sla.end_offset+offset))

        second = [sla.start_offset,  sla.end_offset]
        start_pos_x = [0, 0]
        start_pos_y = [10, 10]
        sla.calc_offset_dist(start_pos_x, start_pos_y)

        # res = sla.calc_slip(start_ang)
        # sla.calc_offset_dist()
        # trj.plot(sla.start_offset_list[0], sla.start_offset_list[1],
        #          ls="--", color="cyan", lw=1, alpha=trj_alpha)
        # trj.plot(res["x"] + sla.turn_offset["x"], res["y"] + + sla.turn_offset["y"], color="blue", lw=1,
        #          alpha=trj_alpha, ls="--")
        # trj.plot(sla.end_offset_list[0], sla.end_offset_list[1],
        #          ls="--", color="cyan", lw=1, alpha=trj_alpha)

        print('{}:'.format(type))
        print('  v: {}'.format(sla.v))
        print('  ang: {}'.format(sla.base_ang))
        print('  rad: {}'.format(sla.rad))
        print('  pow_n: {}'.format(sla.pow_n))
        print('  time: {}'.format(sla.base_time))
        print('  front: {{ left: {}, right: {} }}'.format(
            sla.start_offset, sla.start_offset))
        print('  back: {{ left: {}, right: {} }}'.format(
            sla.end_offset+offset, sla.end_offset+offset))

        third = [sla.start_offset,  sla.end_offset]

        print('front: {}'.format(second[0]-first[0]))
        print('back:{} x sqrt(2)'.format((second[1]-first[1])/math.sqrt(2)))

        trj.set_aspect('1.0')
        plot_range = [-60, 180]

        plt.xlim(plot_range)
        plt.ylim(plot_range)

        plt.show()
