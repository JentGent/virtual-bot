classdef Bot < handle
    properties (Constant)
        AUTO_SPEED = 45
        WALL_DIST = 20
        MANUAL_SPEED = 30
        CLAW_SPEED = 40
        POSE_LOOKBACK = 5
        COLOR_THRESHOLD = 5
    end
    properties
        hw
        odo
        viz
        maze
        
        isRunning = true
        wasStraight = false
        toPhase = "phasePickup"
        phase = "phaseToPickup"
        state = "stateStraight"
        pState = "stateStraight"
        targetAngle = 0
        pDegL = 0
        pDegR = 0
        pRightDist = 0
        pCell = 0
        poses = []
    end

    methods(Static)
        function y = inter(x, ps)
            if x <= ps(1, 1)
                y = ps(1, 2);
                return;
            end
            
            for i = 2:size(ps, 1)
                if x <= ps(i, 1)
                    x0 = ps(i-1, 1);
                    y0 = ps(i-1, 2);
                    x1 = ps(i, 1);
                    y1 = ps(i, 2);
                    y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
                    return;
                end
            end
            y = ps(end, 2);
        end
    end
    
    methods
        function obj = Bot(hw, maze, odo, viz)
            obj.hw = hw;
            obj.maze = maze;
            obj.odo = odo;
            obj.viz = viz;
            viz.bot = obj;

            set(viz.fig, "KeyPressFcn", @obj.onKeyPress);
            set(viz.fig, "KeyReleaseFcn", @obj.onKeyRelease);
        end

        function onKeyPress(obj, src, event)
            hw = obj.hw;
            switch event.Key
                case "uparrow"
                    hw.move(obj.MANUAL_SPEED, obj.MANUAL_SPEED);
                case "downarrow"
                    hw.move(-obj.MANUAL_SPEED, -obj.MANUAL_SPEED);
                case "leftarrow"
                    hw.move(-obj.MANUAL_SPEED/2, obj.MANUAL_SPEED/2);
                case "rightarrow"
                    hw.move(obj.MANUAL_SPEED/2, -obj.MANUAL_SPEED/2);
                case "w"
                    hw.moveClaw(obj.CLAW_SPEED);
                case "s"
                    hw.moveClaw(-obj.CLAW_SPEED);
                case "b"
                    obj.resetAngles();
                    if obj.phase == "phasePickup"
                        obj.phase = "phaseToDropoff";
                    elseif obj.phase == "phaseDropoff"
                        obj.phase = "phaseToStart";
                    end
                    odo = obj.odo;
                    odo.setPose(odo.pose(1), odo.pose(2), obj.targetAngle);
                    obj.changeState("stateStraight");
                    obj.wasStraight = false;
                case "p"
                    hw.stop();
                    obj.isRunning = false;
            end
        end
        function onKeyRelease(obj, src, event)
            obj.hw.stop();
        end

        function start(obj)
            obj.hw.beep();
            obj.hw.resetMotors();
            obj.viz.update(obj);
        end
        function stop(obj)
            obj.isRunning = false;
            obj.hw.stop();
        end

        function changeState(obj, newState)
            obj.pState = obj.state;
            obj.state = newState;
            if newState ~= "stateStraight"
                obj.wasStraight = false;
            end
        end

        function open = openRight(obj, distRight)
            if distRight > obj.maze.CELL_SIZE - obj.hw.xUS
                open = true;
                return;
            end
            BUFFER = obj.maze.CELL_SIZE / 2;
            open = distRight > obj.getDistToGrid(mod(round(obj.targetAngle / 90), 4)) + BUFFER;
        end

        function tf = isInCenter(obj)
            CELL_SIZE = obj.maze.CELL_SIZE;
            switch mod(round(obj.targetAngle / 90), 2)
                case 0 % going horizontal
                    x = mod(obj.odo.pose(1), CELL_SIZE);
                case 1 % going vertical
                    x = mod(obj.odo.pose(2), CELL_SIZE);
            end
            tf = (x > CELL_SIZE * 0.25 && x < CELL_SIZE * 0.75);
        end

        function drift = getDrift(obj)
            odo = obj.odo;
            targetAngle = obj.targetAngle;
            CELL_SIZE = obj.maze.CELL_SIZE;
            FACTOR = 0.5;
            switch mod(round(obj.targetAngle / 90), 4)
                case 0 % going east
                    targetAngle = targetAngle + max(0, mod(odo.pose(2), CELL_SIZE) - CELL_SIZE / 2) * FACTOR;
                case 1 % going south
                    targetAngle = targetAngle + max(0, mod(odo.pose(1), CELL_SIZE) - CELL_SIZE / 2) * FACTOR;
                case 2 % going west
                    targetAngle = targetAngle + max(0, CELL_SIZE - CELL_SIZE / 2 - mod(odo.pose(2), CELL_SIZE)) * FACTOR;
                case 3 % going north
                    targetAngle = targetAngle + max(0, CELL_SIZE - CELL_SIZE / 2 - mod(odo.pose(1), CELL_SIZE)) * FACTOR;
            end
            drift = (odo.pose(3) - max(obj.targetAngle - 5, min(obj.targetAngle + 5, targetAngle)));
        end
        
        function dist = getDistToGrid(obj, dir)
            maze = obj.maze;
            hw = obj.hw;
            CELL_SIZE = maze.CELL_SIZE;
            odo = obj.odo;
            usx = odo.forward(1) * hw.yUS + odo.right(1) * hw.xUS;
            usy = odo.forward(2) * hw.yUS + odo.right(2) * hw.xUS;

            switch dir
                case 0 % going east
                    dist = (mod(odo.pose(2), CELL_SIZE) + usy) / -odo.right(2);
                case 1 % going south
                    dist = (mod(odo.pose(1), CELL_SIZE) + usx) / -odo.right(1);
                case 2 % going west
                    dist = (CELL_SIZE - (mod(odo.pose(2), CELL_SIZE) + usy)) / odo.right(2);
                case 3 % going north
                    dist = (CELL_SIZE - (mod(odo.pose(1), CELL_SIZE) + usx)) / odo.right(1);
            end
        end
        
        function resetAngles(obj)
            angles = obj.hw.getAngles();
            obj.pDegL = angles(1);
            obj.pDegR = angles(2);
        end

        function addPose(obj, pose)
            obj.poses = [obj.poses; pose];
            posesDims = size(obj.poses);
            while posesDims(1) > obj.POSE_LOOKBACK
                obj.poses = obj.poses(2:end, :);
                posesDims = size(obj.poses);
            end
        end

        function run(obj)
            hw = obj.hw;
            hw.resetMotors();

            odo = obj.odo;
            viz = obj.viz;
            maze = obj.maze;

            pAngle = odo.pose(3);
            turnRadius = obj.WALL_DIST;
            wasRed = false;

            while obj.isRunning
                try
                    if obj.phase == "phaseToPickup" || obj.phase == "phaseToDropoff" || obj.phase == "phaseToStart"
                        angles = hw.getAngles();
                        pPose = odo.pose;
                        change = odo.calcChange(angles(1) - obj.pDegL, angles(2) - obj.pDegR);
                        obj.pDegL = angles(1); obj.pDegR = angles(2);
                        odo.applyChange(change);
                        obj.addPose(odo.pose);
        
                        pose = odo.pose;
                        % viz.update();
    
                        color = hw.getColor();
                        if abs(color(2) - color(3)) < obj.COLOR_THRESHOLD && color(1) > max(color(2), color(3)) + obj.COLOR_THRESHOLD % red (pause)
                            if ~wasRed
                                hw.beep();
                                hw.stop();
                                pause(1);
                                odo.applyChange(change);
                                obj.addPose(odo.pose);
                                obj.wasStraight = false;
                                wasRed = true;
                                continue;
                            end
                            wasRed = true;
                        else
                            wasRed = false;
                        end
                        if obj.phase == "phaseToPickup" && obj.state ~= "stateSpin"
                            if color(3) > max(color(1), color(2)) + obj.COLOR_THRESHOLD % blue (pickup)
                                obj.targetAngle = obj.targetAngle + 180;
                                obj.toPhase = "phasePickup";
                                obj.changeState("stateSpin");
                                spinStart = tic;
                            end
                        elseif obj.phase == "phaseToDropoff" && obj.state ~= "stateSpin"
                            if color(2) > max(color(1), color(3)) + obj.COLOR_THRESHOLD % green (dropoff)
                                obj.targetAngle = obj.targetAngle + 180;
                                obj.toPhase = "phaseDropoff";
                                obj.changeState("stateSpin");
                                spinStart = tic;
                            end
                        elseif obj.phase == "phaseToStart"
                            if color(1) > color(3) + obj.COLOR_THRESHOLD && color(2) > color(3) + obj.COLOR_THRESHOLD % yellow (start)
                                obj.isRunning = false;
                                hw.stop();
                                break;
                            end
                        end
                        % disp([obj.phase, obj.state, obj.targetAngle, odo.pose, rand()]);
                        
                        switch obj.state
                            case "stateStraight"
                                rightDist = hw.rightDist();
                                if hw.touchPressed()
                                    hw.stop();
                                    hw.resetMotors();
                                    obj.pDegL = 0; obj.pDegR = 0;
                                    obj.changeState("stateHit");
                                    hw.beep();
                                    continue;
                                end
                                if obj.openRight(rightDist)
                                    if obj.wasStraight && ~wasOpen
                                        odo.passGrid(mod(round(obj.targetAngle / 90), 4), pPose);
                                    end
                                    wasOpen = true;
                                    obj.targetAngle = obj.targetAngle + 90;
                                    cellX = floor(pose(1) / maze.CELL_SIZE);
                                    cellY = floor(pose(2) / maze.CELL_SIZE);
                                    switch mod(round(obj.targetAngle / 90), 2)
                                        case 0
                                            obj.pCell = cellX;
                                        case 1
                                            obj.pCell = cellY;
                                    end
                                    turnRadius = hw.WHEEL_DIST / 2;
                                    obj.changeState("stateRight");
                                    hw.beep();
                                    continue;
                                else
                                    wasOpen = false;
                                end
                                if obj.wasStraight
                                    odo.trackWall(change, obj.pRightDist, rightDist);
                                end
                                drift = obj.getDrift();
                                hw.move(min(max(obj.AUTO_SPEED - drift, 0), obj.AUTO_SPEED), ...
                                        min(max(obj.AUTO_SPEED + drift, 0), obj.AUTO_SPEED));
                                obj.pRightDist = rightDist;
                                obj.wasStraight = true;
                            case "stateHit"
                                odo.setPose(obj.poses(1, 1), obj.poses(1, 2), obj.poses(1, 3));
                                pAngle = odo.pose(3);
                                if rightDist > maze.CELL_SIZE / 2
                                    obj.targetAngle = obj.targetAngle - 180;
                                    obj.changeState("stateReverseBackward");
                                else
                                    obj.targetAngle = obj.targetAngle + 180;
                                    if rightDist < 5
                                        obj.changeState("stateWallBackward")
                                    else
                                        obj.changeState("stateBackward");
                                    end
                                end
                                wallIsVertical = logical(mod(round(1 + obj.targetAngle / 90), 2));
                                odo.correct(0, hw.yTouch, odo.forward, wallIsVertical);
                                hitStart = tic;
                            case "stateReverseBackward"
                                angle = (obj.targetAngle + 180) - odo.pose(3);
                                powerL = Bot.inter(angle, [90, -100; 170, 0; 180, 0; 190, 20]) * obj.AUTO_SPEED / 100;
                                powerR = Bot.inter(angle, [(obj.targetAngle + 180) - pAngle, -80; 90, 100; 170, 20; 180, 0; 190, -20]) * obj.AUTO_SPEED / 100;
                                hw.move(powerL, powerR);
                                if (abs(powerL) < 10 && abs(powerR) < 10) || toc(hitStart) > 4
                                    obj.changeState("stateStraight");
                                    hw.beep();
                                end
                            case "stateBackward"
                                angle = odo.pose(3) - (obj.targetAngle - 180);
                                powerL = Bot.inter(angle, [pAngle - (obj.targetAngle - 180), -80; 90, 100; 170, 20; 180, 0; 190, -20]) * obj.AUTO_SPEED / 100;
                                powerR = Bot.inter(angle, [90, -100; 170, 0; 180, 0; 190, 20]) * obj.AUTO_SPEED / 100;
                                hw.move(powerL, powerR);
                                if (abs(powerL) < 10 && abs(powerR) < 10) || toc(hitStart) > 4
                                    obj.changeState("stateStraight");
                                    hw.beep();
                                end
                            case "stateWallBackward"
                                hw.stop();
                                hw.moveAcc(30, -15);
                                hw.moveAcc(30, 10, 15);
                                hw.moveAcc(30, 5, 0);
                                obj.changeState("stateBackward");
                                hitStart = tic;
                                hw.beep();
                            case "stateRight"
                                obj.changeState("stateForward");
                                hw.beep();
                            case "stateForward"
                                drift = obj.getDrift();
                                hw.move(min(max(obj.AUTO_SPEED - drift, 0), obj.AUTO_SPEED), ...
                                        min(max(obj.AUTO_SPEED + drift, 0), obj.AUTO_SPEED));
                                BUFFER = 0;
                                dir = mod(round(obj.targetAngle / 90), 4);
                                switch dir
                                    case 0 % going east
                                        if odo.pose(1) + hw.yUS + min(0, odo.right(1) * obj.getDistToGrid(dir) / -odo.right(2)) > (obj.pCell + 1) * maze.CELL_SIZE + BUFFER
                                            obj.changeState("stateStraight");
                                            hw.beep();
                                        end
                                    case 1 % going south
                                        if odo.pose(2) - hw.yUS + max(0, odo.right(2) * obj.getDistToGrid(dir) / -odo.right(1)) < obj.pCell * maze.CELL_SIZE - BUFFER
                                            obj.changeState("stateStraight");
                                            hw.beep();
                                        end
                                    case 2 % going west
                                        if odo.pose(1) - hw.yUS + max(0, odo.right(1) * obj.getDistToGrid(dir) / odo.right(2)) < obj.pCell * maze.CELL_SIZE - BUFFER
                                            obj.changeState("stateStraight");
                                            hw.beep();
                                        end
                                    case 3 % going north
                                        if odo.pose(2) + hw.yUS + min(0, odo.right(2) * obj.getDistToGrid(dir) / odo.right(1)) > (obj.pCell + 1) * maze.CELL_SIZE + BUFFER
                                            obj.changeState("stateStraight");
                                            hw.beep();
                                        end
                                end
                            case "stateSpin"
                                angle = odo.pose(3) - (obj.targetAngle - 180);
                                power = Bot.inter(angle, [90, 100; 160, 70; 180, 0; 190, -20]) / 200 * obj.AUTO_SPEED;
                                hw.steer(power, 0);
                                if abs(power) < 10 || toc(spinStart) > 4
                                    hw.stop();
                                    obj.phase = obj.toPhase;
                                    obj.changeState("");
                                    hw.beep();
                                end
                        end
                    elseif obj.phase == "phasePickup"
                        pause(0.04);
                    elseif obj.phase == "phaseDropoff"
                        pause(0.04);
                    end
                catch e
                    disp(e);
                    if isprop(hw, "brick")
                        try
                            hw.stop();
                        catch er
                            disp(er);
                        end
                        DisconnectBrick(hw.brick);
                        hw.setBrick(ConnectBrick("ETAT"));
                    else
                        error("sim");
                    end
                end
            end
            obj.hw.stop();
        end
    end
end