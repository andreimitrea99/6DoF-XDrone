function plot_animation(t, x, params)
% PLOT_ANIMATION 6DoF Trajectory Animation (NED)
% Features:
%   - Precomputes rotation matrices for speed
%   - Reduced circle points
%   - Optional frame skipping
%   - 24 FPS real-time animation
%   - Legends included

%% --- 1. Data Extraction and Setup ---
pos_i = x(:, 11:13); % Inertial Position (x, y, z)
quat  = x(:, 7:10);  % Quaternion
num_steps = length(t);

if num_steps == 0
    warning('Input state matrix (x) is empty. Cannot plot.');
    return;
end

%% --- 2. Interpolation to uniform time (24 FPS) ---
fps = 25;
t_uniform = linspace(t(1), t(end), round(fps*(t(end)-t(1))));

% Interpolate positions and quaternions
pos_i = interp1(t, pos_i, t_uniform, 'linear', 'extrap');
quat  = interp1(t, quat,  t_uniform, 'linear', 'extrap');
t = t_uniform;
num_steps = length(t);

%% --- 3. Frame skipping option (tweakable) ---
frame_skip = 1;  % e.g., 1 = no skip, 2 = skip every other frame
skip_indices = 1:frame_skip:num_steps;

%% --- 4. Body visualization parameters ---
r_magnified = params.radius_visualization;

% --- Reduced circle points for speed ---
num_circle_pts = 25; % you can tweak this
theta = linspace(0, 2*pi, num_circle_pts);
ring_pts_b_frame = r_magnified * [zeros(1,num_circle_pts); cos(theta); sin(theta)];

% Blue/red lines along body z-axis
blue_line_b_vector = [[0;0;0], [0;0;r_magnified]];
red_line_b_vector  = [[0;0;0], [0;0;-r_magnified]];

%% --- 5. Precompute rotation matrices for speed ---
R_all = cell(num_steps,1);
for i = 1:num_steps
    R_all{i} = rotQuat(quat(i,:));
end

%% --- 6. Figure Setup ---
fig_anim = figure('Name','6DoF Trajectory Animation','NumberTitle','off');
ax_anim = axes(fig_anim);
hold(ax_anim,'on'); grid(ax_anim,'on');
xlabel(ax_anim,'North (m)'); ylabel(ax_anim,'East (m)'); zlabel(ax_anim,'Down (m)');
set(ax_anim,'YDir','reverse','ZDir','reverse'); % NED frame
axis(ax_anim,'equal'); view(ax_anim,45,20);

% Plot full trajectory
plot3(ax_anim,pos_i(:,1),pos_i(:,2),pos_i(:,3),'c--','DisplayName','Trajectory');

%% --- 7. Compute extents (your preferred method) ---
x_min = min(pos_i(:,1)); x_max = max(pos_i(:,1));
y_min = min(pos_i(:,2)); y_max = max(pos_i(:,2));
z_min = min(pos_i(:,3)); 
% z_max = max(0,max(pos_i(:,3)));
z_max = max(pos_i(:,3));

x_extent = (x_max - x_min) + 2*r_magnified;
y_extent = (y_max - y_min) + 4*r_magnified;
z_extent = (z_max - z_min) + 4*r_magnified;

x_center = (x_min + x_max)/2;
y_center = (y_min + y_max)/2;
z_center = (z_min + z_max)/2;

set(ax_anim,'XLim',x_center+[-x_extent/2 x_extent/2], ...
             'YLim',y_center+[-y_extent/2 y_extent/2], ...
             'ZLim',z_center+[-z_extent/2 z_extent/2]);

%% --- 8. Initial body graphics ---
R_init = R_all{1};
ringPts_i = R_init * ring_pts_b_frame + pos_i(1,:)';
ring_h = fill3(ax_anim, ringPts_i(1,:), ringPts_i(2,:), ringPts_i(3,:), ...
               'y','FaceAlpha',0.5,'EdgeColor','none','DisplayName','Body Disk');

marker_h = plot3(ax_anim, NaN, NaN, NaN, 'mo','MarkerFaceColor','m','MarkerSize', 1.5*r_magnified,'DisplayName','Body CG');

blue_line_i_init = R_init * blue_line_b_vector + pos_i(1,:)';
blue_line_h = plot3(ax_anim, blue_line_i_init(1,:), blue_line_i_init(2,:), ...
                    blue_line_i_init(3,:),'b-','LineWidth',2,'DisplayName','Top of Body');

red_line_i_init = R_init * red_line_b_vector + pos_i(1,:)';
red_line_h = plot3(ax_anim, red_line_i_init(1,:), red_line_i_init(2,:), ...
                   red_line_i_init(3,:),'r-','LineWidth',2,'DisplayName','Bottom of Body');

title_h = title(ax_anim,sprintf('t = %.2f s',t(1)));

% Add legend
legend(ax_anim,'show','Location','bestoutside');

%% --- 9. Slider & Buttons ---
if num_steps > 1
    slider_h = uicontrol('Parent',fig_anim,'style','slider', ...
        'Min',1,'Max',num_steps,'Value',1,'Position',[250 20 400 20], ...
        'SliderStep',[1/(num_steps-1),10/(num_steps-1)]);
    addlistener(slider_h,'ContinuousValueChange',@(src,~) slider_callback(round(src.Value)));
else
    slider_h = uicontrol('Parent',fig_anim,'style','slider','Visible','off');
end

button_width = 80; button_height = 25; y_pos = 50;
uicontrol('Parent',fig_anim,'Style','pushbutton','String','▶ Play', ...
    'Position',[250 y_pos button_width button_height],'Callback',@(~,~) start_animation());
uicontrol('Parent',fig_anim,'Style','pushbutton','String','❚❚ Pause', ...
    'Position',[340 y_pos button_width button_height],'Callback',@(~,~) pause_animation());
uicontrol('Parent',fig_anim,'Style','pushbutton','String','⇤ Restart', ...
    'Position',[430 y_pos button_width button_height],'Callback',@(~,~) restart_animation());

%% --- 10. Timer (24 FPS) ---
frame_rate = 1/fps;
anim_timer = timer('ExecutionMode','fixedRate', ...
                   'Period',frame_rate, ...
                   'TimerFcn',@(~,~) animate_step(), ...
                   'BusyMode','drop');

set(fig_anim,'DeleteFcn',@(~,~) delete_timer());

current_step = 1;
update_plot(current_step);

%% --- Nested Functions ---

    function update_plot(i)
        i = max(1, min(i, num_steps));
        P_i = pos_i(i,:)';
        R_i_b = R_all{i};

        newRingPts_i = R_i_b * ring_pts_b_frame + P_i;
        newBlueLinePts_i = R_i_b * blue_line_b_vector + P_i;
        newRedLinePts_i  = R_i_b * red_line_b_vector + P_i;

        set(ring_h,'Vertices',newRingPts_i');
        set(marker_h,'XData',P_i(1),'YData',P_i(2),'ZData',P_i(3));
        set(blue_line_h,'XData',newBlueLinePts_i(1,:),'YData',newBlueLinePts_i(2,:),'ZData',newBlueLinePts_i(3,:));
        set(red_line_h,'XData',newRedLinePts_i(1,:),'YData',newRedLinePts_i(2,:),'ZData',newRedLinePts_i(3,:));
        set(title_h,'String',sprintf('t = %.2f s (step %d/%d)',t(i),i,num_steps));

        if num_steps>1, set(slider_h,'Value',i); end
        drawnow limitrate
    end

    function animate_step()
        if strcmp(anim_timer.Running,'on')
            if current_step >= num_steps
                pause_animation();
                set(title_h,'String','Animation complete. Use slider to review.');
                return;
            end
            update_plot(current_step);
            current_step = current_step + frame_skip;
        end
    end

    function start_animation()
        if strcmp(anim_timer.Running,'off')
            if current_step >= num_steps, current_step = 1; end
            start(anim_timer);
        end
    end

    function pause_animation()
        if strcmp(anim_timer.Running,'on'), stop(anim_timer); end
    end

    function restart_animation()
        pause_animation();
        current_step = 1;
        update_plot(current_step);
    end

    function slider_callback(i)
        pause_animation();
        current_step = i;
        update_plot(current_step);
    end

    function delete_timer()
        if isvalid(anim_timer)
            stop(anim_timer);
            delete(anim_timer);
        end
    end
end
