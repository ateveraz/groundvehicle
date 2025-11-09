classdef linearbicycle < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties (Access = public)
        Cf = 160000; % Front cornering stiffness [N/rad]
        Cr = 170000; % Rear cornering stiffness [N/rad]
        lf = 1.2;    % Distance from CG to front axle [m]
        lr = 1.6;    % Distance from CG to rear axle [m]
        m = 1500;    % Vehicle mass [kg]
        Iz = 2250;   % Yaw moment of inertia [kg*m^2]
        mu = 0.01;   % Tire-road friction coefficient
    end



    % Pre-computed constants or internal states
    properties (Access = private)
        A; % State matrix
        B; % Input matrix
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.A = zeros(2,2);
            obj.B = zeros(2,1);

            obj.computeInputMatrix();
        end

        function [fx, ay] = stepImpl(obj, u, delta, x)
            obj.computeStateMatrix(u);
            fx = obj.A * x + obj.B * delta;
            ay = fx(1) + x(2) * u;
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        % Functions to declare outputs
        function [fx, ay] = getOutputSizeImpl(~)
            % Return size for each output port
            fx = [2 1];
            ay = [1 1];
        end

        function [fx, ay] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            fx = 'double';
            ay = 'double';
        end

        function [fx, ay] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            fx = false;
            ay = false;
        end

        function [fx, ay] = isOutputFixedSizeImpl(~)
            % Return true if each output port has a fixed size
            fx = true;
            ay = true;
        end

        function obj = computeStateMatrix(obj, u)
            % Compute the state matrix A based on vehicle parameters
            if u == 0
                obj.A = zeros(2,2);
                return;
            end
            obj.A(1,1) = -(obj.Cf + obj.Cr)/(obj.m * u);
            obj.A(1,2) =  (obj.lr*obj.Cr - obj.lf*obj.Cf)/(obj.m * u) - u;
            obj.A(2,1) =  (obj.lr*obj.Cr - obj.lf*obj.Cf)/(obj.Iz * u);
            obj.A(2,2) = -(obj.lf^2*obj.Cf + obj.lr^2*obj.Cr)/(obj.Iz * u);
        end

        function obj = computeInputMatrix(obj)
            % Compute the input matrix B based on vehicle parameters
            obj.B(1,1) = obj.Cf/obj.m;
            obj.B(2,1) = (obj.lf*obj.Cf)/obj.Iz;
        end
    end
end
