classdef HardwareSim < handle
    properties (Constant)
        MOTOR = struct( ...
            'LEFT', 'A', ...
            'RIGHT', 'D', ...
            'CLAW', 'C' ...
        )
        SENSOR = struct( ...
            'TOUCH', 1, ...
            'COLOR', 2, ...
            'ULTRASONIC', 3, ...
            'GYRO', 4 ...
        )
        WHEEL_RADIUS = 2.75
        WHEEL_DIST = 7.9
        xUS = 5.8
        yUS = -4.2
        yTouch = 9.5
        US_FOV = 10
    end
    properties (SetAccess = immutable)
        REAL_MAZE
    end
    properties
        viz
        realOdo
        distArc
    end
    properties
        leftPower = 0
        rightPower = 0
        degL = 0
        degR = 0
        realDegL = 0
        realDegR = 0
    end

    methods
        function obj = HardwareSim(mazeWidth, mazeHeight)
            obj.REAL_MAZE = Maze(mazeWidth, mazeHeight);
            obj.REAL_MAZE.generate();
            obj.realOdo = Odometry(obj, obj.REAL_MAZE);
            obj.viz = Visualization("Simulation", obj, obj.realOdo, obj.REAL_MAZE);

            while true
                c = randi(obj.REAL_MAZE.MAZE_WIDTH);
                r = randi(obj.REAL_MAZE.MAZE_HEIGHT);            
                dir = randi(4) - 1;
                switch dir
                    case 0
                        if obj.REAL_MAZE.cells(r, c).S == Wall.Blocked
                            break;
                        end
                    case 1
                        if obj.REAL_MAZE.cells(r, c).W == Wall.Blocked
                            break;
                        end
                    case 2
                        if obj.REAL_MAZE.cells(r, c).N == Wall.Blocked
                            break;
                        end
                    case 3
                        if obj.REAL_MAZE.cells(r, c).E == Wall.Blocked
                            break;
                        end
                end
            end
            obj.realOdo.setPose(obj.REAL_MAZE.CELL_SIZE * (c - 0.5) + randn() * 5, obj.REAL_MAZE.CELL_SIZE * (obj.REAL_MAZE.MAZE_HEIGHT + 0.5 - r) + randn() * 5, dir * 90 + randn() * 3);

            obj.distArc = plot(obj.viz.ax, NaN, NaN, 'b-', 'LineWidth', 2);
        end

        function beep(obj)
            disp("beep!");
        end
        function move(obj, left, right)
            obj.leftPower = left;
            obj.rightPower = right;
        end
        function change = moveAcc(obj, speed, leftCM, varargin)
            if isempty(varargin)
                rightCM = leftCM;
            else
                rightCM = varargin{1};
            end
            odo = obj.realOdo;
            dDegL = leftCM * odo.CM_TO_DEG;
            dDegR = rightCM * odo.CM_TO_DEG;
            obj.realDegL = obj.realDegL + dDegL;
            obj.realDegR = obj.realDegR + dDegR;
            change = odo.calcChange(dDegL, dDegR);
            odo.applyChange(change);
        end
        function moveClaw(obj, power)
        end
        function stop(obj)
            obj.leftPower = 0;
            obj.rightPower = 0;
            % obj.brick.WaitForMotor(obj.MOTOR.LEFT);
        end
        function angles = getAngles(obj)
            dt = toc;
            tic;
            pDegL = obj.realDegL;
            pDegR = obj.realDegR;
            obj.realDegL = obj.realDegL + obj.leftPower * (1 + randn() * 0.05) * dt / 100 * obj.WHEEL_RADIUS * 360;
            obj.realDegR = obj.realDegR + obj.rightPower * (1 + randn() * 0.05) * dt / 100 * obj.WHEEL_RADIUS * 360;
            pFakeDegL = obj.degL; pFakeDegR = obj.degR;
            obj.degL = obj.degL + obj.leftPower * dt / 100 * obj.WHEEL_RADIUS * 360;
            obj.degR = obj.degR + obj.rightPower * dt / 100 * obj.WHEEL_RADIUS * 360;
            angles = [round(obj.degL * 0.75 + pFakeDegL * 0.25), round(obj.degR * 0.75 + pFakeDegR * 0.25)];
            
            odo = obj.realOdo;
            change = odo.calcChange(obj.realDegL - pDegL, obj.realDegR - pDegR);
            odo.applyChange(change);

            while obj.colliding()
                d = 0.1;
                odo.setPose(odo.pose(1) - odo.right(1) * d, odo.pose(2) - odo.right(2) * d, odo.pose(3) - d);
            end

            if obj.touchPressed()
                d = 0.1;
                while obj.touchPressed()
                    odo.pose(1) = odo.pose(1) - odo.forward(1) * d;
                    odo.pose(2) = odo.pose(2) - odo.forward(2) * d;
                end
                odo.pose(1) = odo.pose(1) + odo.forward(1) * d;
                odo.pose(2) = odo.pose(2) + odo.forward(2) * d;
            end

            obj.viz.update();

            pause(0.2);
        end
        function steer(obj, speed, radius)
            outerRadius = abs(radius) + obj.WHEEL_DIST / 2;
            innerRadius = abs(radius) - obj.WHEEL_DIST / 2;
            ratio = innerRadius / outerRadius;
            if radius >= 0
                leftPower = speed;
                rightPower = speed * ratio;
            else
                leftPower = speed * ratio;
                rightPower = speed;
            end
            obj.move(leftPower, rightPower);
        end
        function resetMotors(obj)
            obj.degL = 0;
            obj.degR = 0;
        end
        function colliding = colliding(obj)
            odo = obj.realOdo;
            maze = obj.REAL_MAZE;
            US = [odo.pose(1) + odo.right(1)*obj.xUS + odo.forward(1)*obj.yUS, ...
                  odo.pose(2) + odo.right(2)*obj.xUS + odo.forward(2)*obj.yUS];
            colliding = maze.intersecting(US(1), US(2), odo.pose(1), odo.pose(2));
        end
        function pressed = touchPressed(obj)
            odo = obj.realOdo;
            maze = obj.REAL_MAZE;
            touch = [odo.pose(1) + odo.forward(1)*obj.yTouch, ...
                     odo.pose(2) + odo.forward(2)*obj.yTouch];
            pressed = maze.intersecting(touch(1), touch(2), odo.pose(1), odo.pose(2));
        end
        function color = getColor(obj)
            color = [0, 0, 0];
        end
        function dist = rightDist(obj)
            odo = obj.realOdo;
            maze = obj.REAL_MAZE;
            USpos = [odo.pose(1) + odo.right(1)*obj.xUS + odo.forward(1)*obj.yUS, ...
                     odo.pose(2) + odo.right(2)*obj.xUS + odo.forward(2)*obj.yUS];

            dist = 255;
            startAng = odo.pose(3) + 90 - obj.US_FOV/2;
            endAng   = odo.pose(3) + 90 + obj.US_FOV/2;
            stepAng  = obj.US_FOV / 10;
            angles = startAng:stepAng:endAng;

            for ang = angles
                v = [cosd(-ang), sind(-ang)];
                d = maze.rayDistMaze(USpos(1), USpos(2), v(1), v(2));
                dist = min(dist, d);
            end
            dist = min(round(dist * 10) / 10, 255);
            
            xs = []; ys = [];
            for ang = angles
                v = [cosd(-ang), sind(-ang)];
                xs = [xs, USpos(1) + v(1) * dist];
                ys = [ys, USpos(2) + v(2) * dist];
            end

            set(obj.distArc, 'XData', xs, 'YData', ys');
        end
    end
end