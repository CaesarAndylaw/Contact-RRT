classdef Edge 
    properties
        edge
    end
    methods
        function obj = Edge(x_new, x_near)
            obj.edge = [x_new, x_near];
        end
    end
end