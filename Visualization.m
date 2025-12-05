classdef Visualization < handle
    properties
        hw
        odo Odometry
        maze Maze
        bot

        fig
        ax
        robotLine
    end

    methods
        function obj = Visualization(name, hw, odo, maze)
            obj.fig = figure("Name", name, ...
                             "CloseRequestFcn", @obj.onClose);
            obj.ax = axes("Parent", obj.fig);
            hold(obj.ax, "on");
            axis(obj.ax, "equal");
            Gwidth = maze.CELL_SIZE * maze.MAZE_WIDTH;
            Gheight = maze.CELL_SIZE * maze.MAZE_HEIGHT;
            xlim(obj.ax, [0, Gwidth]);
            ylim(obj.ax, [0, Gheight]);
            xticks(obj.ax, 0:maze.CELL_SIZE:Gwidth);
            yticks(obj.ax, 0:maze.CELL_SIZE:Gheight);
            grid(obj.ax, "on");

            obj.robotLine = plot(obj.ax, NaN, NaN, 'b-', 'LineWidth', 2);
            
            obj.hw = hw;
            obj.odo = odo;
            obj.maze = maze;
        end

        function onClose(obj, src, event)
            disp("closing figure");
            delete(obj.fig);
            obj.hw.stop();
        end

        function update(obj)
            odo = obj.odo;
            hw = obj.hw;
            pose = odo.pose;
            right = odo.right;
            xs = [pose(1) - right(1) * hw.WHEEL_DIST/2, pose(1) + odo.forward(1) * hw.yTouch, pose(1) + right(1) * hw.WHEEL_DIST/2];
            ys = [pose(2) - right(2) * hw.WHEEL_DIST/2, pose(2) + odo.forward(2) * hw.yTouch, pose(2) + right(2) * hw.WHEEL_DIST/2];
            set(obj.robotLine, 'XData', xs, 'YData', ys);
            % drawnow limitrate;
        end
    end
end