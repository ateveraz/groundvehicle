classdef LateralCtrl < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties (Access = public)
        Kp = 2; % Proportional gain
        Kd = 1; % Derivative gain
    end


    methods (Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function steer = stepImpl(obj, e1, e2)
            % Error definition as desired - actual
            steer = obj.Kp * e1 + obj.Kd * e2;
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end
    end
end
