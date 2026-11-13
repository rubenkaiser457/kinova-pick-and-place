function animatePickAndPlace(robot, scene, traj, gripperHasVisual, exportMedia, mediaFolder)
%ANIMATEPICKANDPLACE Animate the trajectory and optionally export GIF/MP4.

    fig = figure('Name', 'Kinova Gen3 pick and place', 'Color', 'w', ...
                 'Position', [100 100 800 600]);
    ax = axes(fig);
    hold(ax, 'on');
    ax.Toolbar.Visible = 'off';   % keep the axes toolbar out of the exported frames
    disableDefaultInteractivity(ax);

    % Static scene
    drawObject(scene.floor,    ax, [0.92 0.92 0.92]);
    drawObject(scene.table,    ax, [0.85 0.70 0.45]);
    drawObject(scene.obstacle, ax, [0.80 0.30 0.30]);
    plot3(ax, traj.targetXY(1), traj.targetXY(2), scene.tableTop + 0.001, 'kx', ...
          'MarkerSize', 12, 'LineWidth', 2);

    if gripperHasVisual
        showCollisions = 'off';
    else
        showCollisions = 'on';
    end

    if exportMedia
        if ~exist(mediaFolder, 'dir'), mkdir(mediaFolder); end
        gifFile = fullfile(mediaFolder, 'pick_and_place.gif');
        video = VideoWriter(fullfile(mediaFolder, 'pick_and_place.mp4'), 'MPEG-4');
        video.FrameRate = 30;
        open(video);
        gifStarted = false;
    end

    objectPatch = [];
    n = size(traj.Q, 1);
    for i = 1:n
        if ~isvalid(fig)
            warning("pickplace:animation", "Figure was closed, animation stopped at sample %d.", i);
            break;
        end
        show(robot, traj.Q(i, :), 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, ...
             'Visuals', 'on', 'Collisions', showCollisions, 'Frames', 'off');

        % Clicking or hovering a robot body would highlight it and show a
        % label in the exported frames, so drawn objects ignore the mouse
        set(findobj(ax, '-property', 'HitTest'), 'HitTest', 'off', 'PickableParts', 'none');

        if ~isempty(objectPatch), delete(objectPatch); end
        obj = scene.object;
        obj.Pose = traj.objectPose(:, :, i);
        objectPatch = drawObject(obj, ax, [1.0 0.55 0.0]);

        axis(ax, 'equal');
        xlim(ax, [-0.3 1.0]); ylim(ax, [-0.7 0.7]);
        zlim(ax, [scene.tableTop - 0.1, scene.tableTop + 1.0]);
        view(ax, 135, 25);
        grid(ax, 'on');
        xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
        title(ax, sprintf('Kinova Gen3 pick and place: %s', traj.phaseNames(traj.phase(i))));
        drawnow limitrate;

        if exportMedia && isvalid(fig)
            frame = getframe(fig);
            writeVideo(video, frame);
            if mod(i, 3) == 1          % every 3rd frame keeps the GIF small
                [img, map] = rgb2ind(frame2im(frame), 128);
                if ~gifStarted
                    imwrite(img, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.1);
                    gifStarted = true;
                else
                    imwrite(img, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
                end
            end
        end
    end

    if exportMedia
        close(video);
        fprintf('Media written to %s\n', mediaFolder);
    end
end

function p = drawObject(obj, ax, color)
    [~, p] = show(obj, 'Parent', ax);
    p.FaceColor = color;
    p.EdgeColor = 'none';
end
