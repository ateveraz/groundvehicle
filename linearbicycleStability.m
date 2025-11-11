classdef linearbicycleStability < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    properties (Access = private)
        Cf = 160000; % Front cornering stiffness [N/rad]
        Cr = 170000; % Rear cornering stiffness [N/rad]
        lf = 1.2;    % Distance from CG to front axle [m]
        lr = 1.6;    % Distance from CG to rear axle [m]
        m = 3000;    % Vehicle mass [kg]
        Iz = 2250;   % Yaw moment of inertia [kg*m^2]
        mu = 0.01;   % Tire-road friction coefficient
    end



    % Pre-computed constants or internal states
    properties (Access = private)
        A; % State matrix
        B_delta; % Input matrix for steering angle
        B_Mz;    % Input matrix for yaw moment
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.A = zeros(2,2);
            obj.B_delta = zeros(2,1);
            obj.B_Mz = zeros(2,1);

            modelParameters = evalin('base', 'model');

            obj.Cf = modelParameters.Cf;
            obj.Cr = modelParameters.Cr;
            obj.lf = modelParameters.lf;
            obj.lr = modelParameters.lr;
            obj.m  = modelParameters.m;
            obj.Iz = modelParameters.Iz;
            obj.mu = modelParameters.mu;

            obj.computeInputMatrix();
        end

        function [ay, fx] = stepImpl(obj, Vx, delta, Mz, x)
            obj.computeStateMatrix(Vx);
            fx = obj.A * x + obj.B_delta * delta + obj.B_Mz * Mz;
            ay = fx(1) + x(2) * Vx;
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        % Functions to declare outputs
        function [ay, fx] = getOutputSizeImpl(~)
            % Return size for each output port
            ay = [1 1];
            fx = [2 1];
        end

        function [ay, fx] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            ay = 'double';
            fx = 'double';
        end

        function [ay, fx] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            ay = false;
            fx = false;
        end

        function [ay, fx] = isOutputFixedSizeImpl(~)
            % Return true if each output port has a fixed size
            ay = true;
            fx = true;
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
            obj.B_delta(1,1) = obj.Cf/obj.m;
            obj.B_delta(2,1) = (obj.lf*obj.Cf)/obj.Iz;

            % Input matrix for yaw moment Mz
            obj.B_Mz(1,1) = 0;
            obj.B_Mz(2,1) = 1/obj.Iz; % Mz directly affects 'r'
        end
    end
end
