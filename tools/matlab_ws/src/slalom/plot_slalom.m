function [l_start, l_end] = plot_slalom(turn_mode, tmp_x_list, tmp_y_list, tmp_w_list, ...
        tmp_x, tmp_y, target_angle, is_dia_mode, fig_num, wall_off_offset, wall_off_offset_dia)
    Large = 1; Orval = 2; Dia45 = 3; Dia135 = 4; Dia90 = 5; Dia180 = 6;
    output = 0;
    fig1 = figure('Name',fig_num);
    clf(fig1);
    l_start = 0;
    l_end = 0;

    LineWidth = 2;

    if turn_mode == Large
        [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, 180 - wall_off_offset, 180 - wall_off_offset, target_angle);
        l_start = l_start + wall_off_offset;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) + l_start;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end) + l_end;
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;

    elseif turn_mode == Orval
        l_start = 10 + wall_off_offset;
        l_end = l_start + tmp_x_list(end) - wall_off_offset;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) +l_start - l_end;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end);
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;
    elseif turn_mode == Dia45

        if ~is_dia_mode
            [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, 180 - wall_off_offset_dia * sin(pi / 4), 90 - wall_off_offset_dia * sin(pi / 4), target_angle);
            %l_start = l_start + wall_off_offset;
            plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
            hold on;
            plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start;
            l_end_x1 = tmp_x_list(end) + l_start + l_end * sin(pi / 4);
            l_end_y0 = tmp_y_list(end);
            l_end_y1 = tmp_y_list(end) + l_end * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset;
        else
            l_start = (90 - tmp_x_list(end)) / sin(pi / 4);
            l_end = 180 - tmp_y_list(end) - l_start * sin(pi / 4) - wall_off_offset;

            plot(tmp_x_list + l_start * cos(pi / 4), tmp_y_list + l_start * sin(pi / 4), 'LineWidth', LineWidth);
            hold on;
            l_start_x1 = l_start * sin(pi / 4);
            l_start_y1 = l_start * sin(pi / 4);
            plot([-wall_off_offset_dia * cos(pi / 4) l_start_x1], [-wall_off_offset_dia * sin(pi / 4) l_start_y1], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_x1 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_y0 = tmp_y_list(end) + l_start * sin(pi / 4);
            l_end_y1 = tmp_y_list(end) + l_start * sin(pi / 4) + l_end;
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;

            l_start = l_start + wall_off_offset_dia;
        end

    elseif turn_mode == Dia135

        if ~is_dia_mode
            [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, 90 + wall_off_offset_dia * sin(pi / 4), 180 - wall_off_offset_dia * sin(pi / 4), target_angle);
            plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
            hold on;
            plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start;
            l_end_x1 = tmp_x_list(end) + l_start - l_end * sin(pi / 4);
            l_end_y0 = tmp_y_list(end);
            l_end_y1 = tmp_y_list(end) + l_end * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset;
        else
            l_start = (180 - tmp_y_list(end)) / sin(pi / 4);
            l_end = abs(-90 - tmp_x_list(end) - l_start * sin(pi / 4) + wall_off_offset);

            plot(tmp_x_list + l_start * sin(pi / 4), tmp_y_list + l_start * sin(pi / 4), 'LineWidth', LineWidth);
            hold on;
            l_start_x1 = l_start * sin(pi / 4);
            l_start_y1 = l_start * sin(pi / 4);
            plot([-wall_off_offset_dia * cos(pi / 4) l_start_x1], [-wall_off_offset_dia * cos(pi / 4) l_start_y1], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_x1 = tmp_x_list(end) + l_start * sin(pi / 4) - l_end;
            l_end_y0 = tmp_y_list(end) + l_start * sin(pi / 4);
            l_end_y1 = tmp_y_list(end) + l_start * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset_dia;
        end

    elseif turn_mode == Dia90
        [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, 90 * sqrt(2) - wall_off_offset_dia, 90 * sqrt(2) - wall_off_offset_dia, target_angle);
        l_start = l_start + wall_off_offset_dia;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset_dia l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) + l_start;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end) + l_end;
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;

    end

    if turn_mode == Dia90
        xlim([-40 200]);
        ylim([-40 200]);
        hold on;

        plot([-270 270], [0 0], 'k:');
        % plot([-270 270], [180 180], 'k:');
        tmp_dist = 90 * sqrt(2);
        plot([-270 270], [tmp_dist tmp_dist], 'k:');

        plot([0 0], [-90 270], 'k:');
        plot([tmp_dist tmp_dist], [-90 270], 'k:');

        plot([-tmp_dist / 2 -tmp_dist / 2], [-180 270], 'k:');
        plot([tmp_dist / 2 tmp_dist / 2], [-180 270], 'k:');

        plot([-90 360], [-90 360], 'k:');

        hold off;
    elseif ~is_dia_mode
        xlim([-40 220]);
        ylim([-40 220]);
        hold on;

        plot([-90 270], [0 0], 'k:');
        plot([-90 270], [180 180], 'k:');

        plot([0 0], [-90 270], 'k:');
        plot([180 180], [-90 270], 'k:');

        plot([0 360], [-90 270], 'k:');
        plot([0 360], [270 -90], 'k:');

        plot([-90 270], [84 84], 'r:');
        plot([-90 270], [96 96], 'r:');

        plot([84 84], [-90 270], 'r:');
        plot([96 96], [-90 270], 'r:');

        hold off;
    else
        xlim([-120 180]);
        ylim([-90 270]);

        hold on;

        plot([-90 270], [0 0], 'k:');
        plot([-90 270], [180 180], 'k:');

        plot([0 0], [-90 270], 'k:');
        plot([180 180], [-90 270], 'k:');

        plot([-90 -90], [-180 270], 'k:');
        plot([90 90], [-180 270], 'k:');

        plot([-90 360], [-90 360], 'k:');
        plot([-90 270], [270 -90], 'k:');

        plot([-180 6], [84 84], 'r:');
        plot([-180 6], [96 96], 'r:');

        plot([6 6], [84 96], 'r:');

        plot([174 174], [-90 270], 'r:');
        plot([186 186], [-90 270], 'r:');

        plot([-90 270], [264 264], 'r:');
        plot([-90 270], [276 276], 'r:');

        hold off;
    end

    fprintf('%0.8f\t%0.8f\t\r\n', l_start, l_end);

end
