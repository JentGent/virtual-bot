classdef Maze < handle
    properties (SetAccess = immutable)
        MAZE_WIDTH
        MAZE_HEIGHT
        TOTAL_CELLS
        CELL_SIZE = 58.4
    end
    properties
        cells
    end
    properties (Constant)
        % DIRS = struct('N',[0,1],'S',[0,-1],'E',[1,0],'W',[-1,0]);
        OPPOSITE = struct('N','S','S','N','E','W','W','E');
    end

    methods (Static)
        % from https://stackoverflow.com/questions/9043805/test-if-two-lines-intersect-javascript-function
        function tf = intersects(a, b, c, d, p, q, r, s)
            EPS = 1e-12;
            det = (c - a) * (s - q) - (r - p) * (d - b);
            if abs(det) < EPS
                tf = false;
                return;
            end
            lambda = ((s - q) * (r - a) + (p - r) * (s - b)) / det;
            gamma  = ((b - d) * (r - a) + (c - a) * (s - b)) / det;
            tf = (-EPS <= lambda && lambda <= 1 + EPS) && (-EPS <= gamma && gamma <= 1 + EPS);
        end

        function dist = rayDist(x0, y0, vx, vy, x1, y1, x2, y2)
            EPS = 1e-12;
            dx = x2 - x1;
            dy = y2 - y1;
            det = vx * dy - vy * dx;
            if abs(det) < EPS
                dist = 255;
                return;
            end
            t = ((x1 - x0) * dy - (y1 - y0) * dx) / det;
            u = ((x1 - x0) * vy - (y1 - y0) * vx) / det;
            if (t >= -EPS) && (u >= -EPS) && (u <= 1 + EPS)
                % dist = min(t * hypot(vx, vy), 255);
                dist = min(t, 255); % assume <vx, vy> is a unit vector
            else
                dist = 255;
            end
        end
    end
    
    methods
        function obj = Maze(width, height)
            obj.MAZE_WIDTH = width;
            obj.MAZE_HEIGHT = height;
            obj.TOTAL_CELLS = width * height;
            obj.initCells();
        end

        function dist = rayDistMaze(obj, x, y, vx, vy)
            dist = 255;
            for i = 1:obj.MAZE_WIDTH
                for j = 1:obj.MAZE_HEIGHT
                    c = obj.cells(j, i);
                    if c.W == Wall.Blocked
                        d = Maze.rayDist(x, y, vx, vy, ...
                            (i-1)*obj.CELL_SIZE, (obj.MAZE_HEIGHT+1-j)*obj.CELL_SIZE, ...
                            (i-1)*obj.CELL_SIZE, (obj.MAZE_HEIGHT-j)*obj.CELL_SIZE);
                        dist = min(dist, d);
                    end
                    if c.S == Wall.Blocked
                        d = Maze.rayDist(x, y, vx, vy, ...
                            (i-1)*obj.CELL_SIZE, (obj.MAZE_HEIGHT-j)*obj.CELL_SIZE, ...
                            i*obj.CELL_SIZE, (obj.MAZE_HEIGHT-j)*obj.CELL_SIZE);
                        dist = min(dist, d);
                    end
                end
            end
            d = Maze.rayDist(x, y, vx, vy, ...
                obj.CELL_SIZE * obj.MAZE_WIDTH, 0, ...
                obj.CELL_SIZE * obj.MAZE_WIDTH, obj.CELL_SIZE * obj.MAZE_HEIGHT);
            dist = min(dist, d);
            d = Maze.rayDist(x, y, vx, vy, ...
                0, obj.CELL_SIZE * obj.MAZE_HEIGHT, ...
                obj.CELL_SIZE * obj.MAZE_WIDTH, obj.CELL_SIZE * obj.MAZE_HEIGHT);
            dist = min(dist, d);
        end
        function initCells(obj)
            obj.cells = repmat(Cell(1, 1), obj.MAZE_HEIGHT, obj.MAZE_WIDTH);
            for row = 1:obj.MAZE_HEIGHT
                for col = 1:obj.MAZE_WIDTH
                    obj.cells(row, col) = Cell(col, row);
                end
            end
        end
        function neighbors = getNeighbors(obj, cell)
            neighbors = repmat(struct('dir', '', 'neighbor', obj.cells(1, 1)), 1, 4);
            n = 0;
            if cell.row > 1
                n = n + 1;
                neighbors(n).dir = 'N';
                neighbors(n).neighbor = obj.cells(cell.row - 1, cell.col);
            end
            if cell.row < obj.MAZE_HEIGHT
                n = n + 1;
                neighbors(n).dir = 'S';
                neighbors(n).neighbor = obj.cells(cell.row + 1, cell.col);
            end
            if cell.col > 1
                n = n + 1;
                neighbors(n).dir = 'W';
                neighbors(n).neighbor = obj.cells(cell.row, cell.col - 1);
            end
            if cell.col < obj.MAZE_WIDTH
                n = n + 1;
                neighbors(n).dir = 'E';
                neighbors(n).neighbor = obj.cells(cell.row, cell.col + 1);
            end
            neighbors = neighbors(1:n);
        end
        function generate(obj)
            for r = 1:obj.MAZE_HEIGHT
                for c = 1:obj.MAZE_WIDTH
                    cell = obj.cells(r, c);
                    cell.reset();
                    cell.blockAll();
                end
            end

            obj.cells(randi(obj.MAZE_HEIGHT), randi(obj.MAZE_WIDTH)).visited = true;
            visited = 1;
            
            while visited < obj.TOTAL_CELLS
                while true
                    r0 = randi(obj.MAZE_HEIGHT);
                    c0 = randi(obj.MAZE_WIDTH);
                    cell = obj.cells(r0, c0);
                    if ~cell.visited
                        break;
                    end
                end

                path = Cell.empty(0, 1);
                dirs = struct('dir', {}, 'neighbor', {});
                
                while ~cell.visited
                    loopStart = find(path == cell, 1);
                    if ~isempty(loopStart)
                        path(loopStart:end) = [];
                        dirs(loopStart:end) = [];
                    end
                    path(end + 1) = cell;
                    neighbors = obj.getNeighbors(cell);
                    dirCell = neighbors(randi(numel(neighbors)));
                    dirs(end + 1) = dirCell;
                    cell = dirCell.neighbor;
                end

                for i = 1:numel(path)
                    cell = path(i);
                    nextCell = dirs(i).neighbor;
                    d = dirs(i).dir;
                    cell.(d) = Wall.Open;
                    nextCell.(obj.OPPOSITE.(d)) = Wall.Open;
                    cell.visited = true;
                    visited = visited + 1;
                end
            end
        end
        function draw(obj, ax, varargin)
            if ~exist('ax', 'var')
                ax = gca;
            end
            
            lineStyle = '-';
            lineColor = 'red';
            for i = 1:2:numel(varargin)
                name = varargin{i};
                val = varargin{i+1};
                switch lower(name)
                    case 'linestyle', lineStyle = val;
                    case 'linecolor', lineColor = val;
                end
            end
                    

            wasHeld = ishold(ax);
            hold(ax, 'on');

            offx = 0; offy = 0;
            sx = obj.CELL_SIZE;
            sy = obj.CELL_SIZE;
            lineWidth = 2;

            for r = 1:obj.MAZE_HEIGHT
                for c = 1:obj.MAZE_WIDTH
                    x0 = offx + (c-1) * sx;
                    y0 = offy + (obj.MAZE_HEIGHT-r) * sy;
                    cell = obj.cells(r, c);

                    if cell.N == Wall.Blocked
                        line([x0, x0+sx], [y0+sy, y0+sy], 'LineWidth', lineWidth, 'LineStyle', lineStyle, 'Color', lineColor, 'Parent', ax);
                    end
                    if cell.S == Wall.Blocked
                        line([x0, x0+sx], [y0, y0], 'LineWidth', lineWidth, 'LineStyle', lineStyle, 'Color', lineColor, 'Parent', ax);
                    end
                    if cell.W == Wall.Blocked
                        line([x0, x0], [y0, y0+sy], 'LineWidth', lineWidth, 'LineStyle', lineStyle, 'Color', lineColor, 'Parent', ax);
                    end
                    if cell.E == Wall.Blocked
                        line([x0+sx, x0+sx], [y0, y0+sy], 'LineWidth', lineWidth, 'LineStyle', lineStyle, 'Color', lineColor, 'Parent', ax);
                    end
                end
            end

            if ~wasHeld
                hold(ax, 'off');
            end
        end
    end
end