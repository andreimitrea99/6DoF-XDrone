function status = live_dx_plot(t, y, flag, fig)
% Live plotting function for dx components during ODE45 integration

status = 0; % always continue integration

if strcmp(flag, 'init')
    % nothing special at init
elseif isempty(flag)
    params = getappdata(fig, 'params');
    h = getappdata(fig, 'h');

    for i = 1:length(t)
        dx = sixdof_wrapper(t(i), y(:,i), params);
        % Update each subplot
        for j = 1:min(length(dx), numel(h))
            xdata = get(h(j), 'XData');
            ydata = get(h(j), 'YData');
            set(h(j), 'XData', [xdata, t(i)], 'YData', [ydata, dx(j)]);
        end
    end
    drawnow limitrate nocallbacks
elseif strcmp(flag, 'done')
    disp('Integration finished.');
end
end
