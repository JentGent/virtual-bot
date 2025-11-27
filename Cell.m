classdef Cell < handle
    properties
        col
        row
        N = Wall.Unknown
        S = Wall.Unknown
        E = Wall.Unknown
        W = Wall.Unknown
        visited = false
    end

    methods
        function obj = Cell(col, row)
            obj.col = col;
            obj.row = row;
        end

        function blockAll(obj)
            obj.N = Wall.Blocked;
            obj.S = Wall.Blocked;
            obj.E = Wall.Blocked;
            obj.W = Wall.Blocked;
        end
        function reset(obj)
            obj.N = Wall.Unknown;
            obj.S = Wall.Unknown;
            obj.E = Wall.Unknown;
            obj.W = Wall.Unknown;
            obj.visited = false;
        end
    end
end