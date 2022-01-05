% ego
dt = 0.001/4;
Normal = 0;Large = 1; Orval = 2; Dia45 = 3; Dia135 = 4; Dia90 = 5; Dia180 = 6;

ego_v = 500;
turn_mode = Large;

is_dia_mode = false;
slip_gain = -495;

wall_off_offset = 0;
wall_off_offset_dia = 0 * sqrt(2);

if turn_mode == Normal
    radius = 40; sla.pow_n = 4;
    target_angle = 90, is_dia_mode = false, turn_mode_str = 'Normal';
elseif turn_mode == Large
    radius = 65; sla.pow_n = 4;
    target_angle = 90, is_dia_mode = false, turn_mode_str = 'Large';
elseif turn_mode == Orval
    radius = 86.30025; sla.pow_n = 4;
    target_angle = 180, is_dia_mode = false, turn_mode_str = 'Orval';
elseif turn_mode == Dia45
    radius = 145; sla.pow_n = 4;
    target_angle = 45, turn_mode_str = 'Dia45';
elseif turn_mode == Dia135
    radius = 85; sla.pow_n = 4;
    target_angle = 135, turn_mode_str = 'Dia135';
elseif turn_mode == Dia90
    is_dia_mode = false, turn_mode_str = 'Dia90';
end

tmp_x = 0;
tmp_y = 0;

start_offset_idx = 0;
end_offset_idx = 0;

if is_dia_mode
    fprintf('start_point_s = %0.8f\r\n', sqrt(tmp_x^2 +tmp_y^2));
end

start_theta = 0;

if is_dia_mode
    start_theta = 45 * pi / 180;
end

alphaTemp = ego_v / radius;
sla.base_alpha = alphaTemp;
sla.counter = int32(0);

tmp_w = (0);
tmp_dist = (0);

Et = 0;

if sla.pow_n == 2
    Et = 0.603450161218938087668;
elseif sla.pow_n == 4
    Et = 0.763214618198974433973;
end

sla.base_time = calc_slalom(ego_v, radius, Et, target_angle * pi / 180);
sla.limit_time_count = sla.base_time * 2 / dt;
% disp(sla.base_time);
fprintf('%.8f\r\n', sla.base_time);

tmp_x_list = zeros(100, 1);
tmp_y_list = zeros(100, 1);
tmp_w_list = zeros(100, 1);

tmp_slip_x_list = zeros(100, 1);
tmp_slip_y_list = zeros(100, 1);

tmp_x_list(1) = tmp_x;
tmp_y_list(1) = tmp_y;
slip_tmp_x = tmp_x;
slip_tmp_y = tmp_y;

slip_theta = 0;
tmp_theta = 0;

for i = 2:1:sla.limit_time_count + 1

    if turn_mode == Dia90

        if tmp_x_list(i - 1) <= 0 && i > 10
            break;
        end

    end

    %tmp_alpha = alphaTemp * calc_neipire(dt * (i - 1), sla.base_time, sla.pow_n);
    tmp_alpha = alphaTemp * calc_neipire(dt * (i - 1 + start_offset_idx), sla.base_time, sla.pow_n);
    tmp_w = tmp_w + tmp_alpha * dt;
    tmp_theta = tmp_theta + tmp_w * dt;
    tmp_x = tmp_x + ego_v * cos(start_theta + tmp_theta) * dt;
    tmp_y = tmp_y + ego_v * sin(start_theta + tmp_theta) * dt;
    tmp_x_list(i) = (tmp_x);
    tmp_y_list(i) = (tmp_y);
    tmp_w_list(i) = (tmp_w);

    slip_theta = (1 / dt * slip_theta -tmp_w) / (1 / dt + slip_gain / (ego_v / 1000));

    slip_tmp_x = slip_tmp_x + ego_v * cos(start_theta + tmp_theta + slip_theta) * dt;
    slip_tmp_y = slip_tmp_y + ego_v * sin(start_theta + tmp_theta + slip_theta) * dt;
    tmp_slip_x_list(i) = (slip_tmp_x);
    tmp_slip_y_list(i) = (slip_tmp_y);

    if i > sla.limit_time_count - start_offset_idx - end_offset_idx
        break;
    end

end

G = 9.81;
fprintf('pos(x,y,rad,deg,max_G) = (%0.8f, %0.8f, %0.8f, %0.8f, %0.8fG)\r\n', tmp_x, tmp_y, tmp_theta, tmp_theta * 180 / pi, max(tmp_w_list)^2 * (radius / 1000) / G);

[a,b]=plot_slalom(turn_mode, tmp_x_list, tmp_y_list, tmp_w_list, tmp_x, tmp_y, target_angle, is_dia_mode, 'normal', wall_off_offset, wall_off_offset_dia);


fprintf('no_slip %0.8f\t%0.8f\r\n', ...
    a, b);
[l_start, l_end] = plot_slalom(turn_mode, tmp_slip_x_list, tmp_slip_y_list, tmp_w_list, tmp_x, tmp_y, target_angle, is_dia_mode, 'slip', wall_off_offset, wall_off_offset_dia);

fprintf('%d\t%s\t', ...
    ego_v, turn_mode_str);

fprintf('%d\t%d\t', ...
    target_angle, radius);

fprintf('%0.8f\t%0.8f\t', ...
    l_start, l_end);

fprintf('%0.8f\t%0.8f\t', ...
    l_start, l_end);

fprintf('%0.8f\t%d\t', ...
    sla.base_time, sla.pow_n);

fprintf('%0.8f\t%0.8f\t%0.8f\r\n', ...
    l_start, l_end, l_start);
