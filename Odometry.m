classdef Odometry < handle
    properties (Constant)
        MAX_ANGLE_CORRECTION = 2
        MAX_POS_CORRECTION = 5
    end
    properties (SetAccess = immutable)
        DEG_TO_CM
        CM_TO_DEG
        DARC_TO_DEG
        RAT_TO_CM
    end
    properties
        hw
        maze
        pose = [0, 0, 0] % [x, y, angle]
        forward = [1, 0]
        right = [0, -1]
    end

    methods
        function obj = Odometry(hw, maze)
            obj.hw = hw;
            obj.maze = maze;
            obj.DEG_TO_CM = pi * hw.WHEEL_RADIUS / 180;
            obj.CM_TO_DEG = 180 / (pi * hw.WHEEL_RADIUS);
            obj.DARC_TO_DEG = 180 / (pi * hw.WHEEL_DIST);
            obj.RAT_TO_CM = hw.WHEEL_DIST / 2;
        end
        
        function change = calcChange(obj, dDegL, dDegR)
            arguments
                obj Odometry
                dDegL (1,1) double
                dDegR (1,1) double
            end

            dArcL = dDegL * obj.DEG_TO_CM;
            dArcR = dDegR * obj.DEG_TO_CM;
            if abs(dArcL - dArcR) < 0.01
                change = [0, (dArcL+dArcR)/2, 0, 1, 0];
                return;
            end
            theta = (dArcL - dArcR) * obj.DARC_TO_DEG;
            R = (dArcL + dArcR) / (dArcL - dArcR) * obj.RAT_TO_CM;
            c = cosd(theta); s = sind(theta);
            change = [R*(1-c), R*s, theta, c, s];
        end

        function setPose(obj, x, y, angle)
            obj.pose = [x, y, angle];
            obj.forward = [cosd(angle), -sind(angle)];
            obj.right = [obj.forward(2), -obj.forward(1)];
        end

        function applyChange(obj, change)
            pose = obj.pose;
            forward = obj.forward;
            right = obj.right;
            obj.pose(1) = pose(1) + right(1) * change(1) + forward(1) * change(2);
            obj.pose(2) = pose(2) + right(2) * change(1) + forward(2) * change(2);
            obj.pose(3) = pose(3) + change(3);
            obj.forward(1) = forward(1) * change(4) + forward(2) * change(5);
            obj.forward(2) = forward(2) * change(4) - forward(1) * change(5);
            obj.right(1) = obj.forward(2);
            obj.right(2) = -obj.forward(1);
        end

        function correct(obj, x, y, axis, wallIsVertical)
            arguments
                obj Odometry
                x (1,1) double
                y (1,1) double
                axis (1,2) double
                wallIsVertical (1,1) logical
            end
            CELL_SIZE = obj.maze.CELL_SIZE;
            if wallIsVertical
                pointX = obj.pose(1) + obj.right(1) * x + obj.forward(1) * y;
                wallX = round(pointX / CELL_SIZE) * CELL_SIZE;
                dx = wallX - pointX;
                obj.pose(1) = obj.pose(1) + dx;
                obj.pose(2) = obj.pose(2) + axis(2) * dx / axis(1);
            else
                pointY = obj.pose(2) + obj.right(2) * x + obj.forward(2) * y;
                wallY = round(pointY / CELL_SIZE) * CELL_SIZE;
                dy = wallY - pointY;
                obj.pose(1) = obj.pose(1) + axis(1) * dy / axis(2);
                obj.pose(2) = obj.pose(2) + dy;
            end
        end

        function passGrid(obj, dir, pPose)
            maze = obj.maze;
            % assume the bot is ~straight
            pose = obj.pose;
            EPS = 1e-12;
            newPose = pose;
            switch dir
                case 0 % going east (+x)
                    wallX = round(pose(1) / maze.CELL_SIZE) * maze.CELL_SIZE;
                    newPose(1) = max(wallX + EPS, min(wallX + pose(1) - pPose(1), pose(1)));
                case 1 % going north (+y)
                    wallY = round(pose(2) / maze.CELL_SIZE) * maze.CELL_SIZE;
                    newPose(2) = max(wallY + EPS, min(wallY + pose(2) - pPose(2), pose(2)));
                case 2 % going west (-x)
                    wallX = round(pose(1) / maze.CELL_SIZE) * maze.CELL_SIZE;
                    newPose(1) = min(wallX - EPS, max(wallX + pose(1) - pPose(1), pose(1)));
                case 3 % going south (-y)
                    wallY = round(pose(2) / maze.CELL_SIZE) * maze.CELL_SIZE;
                    newPose(2) = min(wallY - EPS, max(wallY + pose(2) - pPose(2), pose(2)));
            end
            if abs(newPose(1) - pose(1)) < obj.MAX_POS_CORRECTION && abs(newPose(2) - pose(2)) < obj.MAX_POS_CORRECTION
                obj.pose = newPose;
            end
        end
        
        function trackWall(obj, change, pRightDist, rightDist)
            if abs(rightDist - pRightDist) > 5
                return;
            end
            hw = obj.hw;
            right = [change(4), -change(5)];
            forward = [-right(2), right(1)];
            wall = [hw.xUS + rightDist, hw.yUS];
            pWall = [change(1) + right(1) * (hw.xUS + pRightDist) - forward(1) * hw.yUS, ...
                    -change(2) - right(2) * (hw.xUS + pRightDist) + forward(2) * hw.yUS];
            dWallX = wall(1) - pWall(1); dWallY = wall(2) - pWall(2);
            sqDist = dWallX * dWallX + dWallY * dWallY;
            if sqDist > 2 * 2 && sqDist < 10 * 10
                wallAngle = asind(dWallX / sqrt(sqDist));
                if dWallY < 0
                    wallAngle = -wallAngle;
                end
                pPose = obj.pose(3);
                dir = round(pPose / 90);
                newAngle = dir * 90 - wallAngle;
                obj.pose(3) = max(obj.pose(3) - obj.MAX_ANGLE_CORRECTION, min(obj.pose(3) + obj.MAX_ANGLE_CORRECTION, newAngle));
                obj.forward = [cosd(newAngle), -sind(newAngle)];
                obj.right = [obj.forward(2), -obj.forward(1)];
                obj.correct(hw.xUS + rightDist, hw.yUS, obj.right, logical(mod(dir, 2)));
            end
        end
    end
end