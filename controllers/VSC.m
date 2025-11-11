classdef VSC < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties (Access = public)
        Kp = 2; % Proportional gain
        Kd = 1; % Derivative gain
    end

    properties (Access = private)
        gravity = 9.81; % Gravity constant [m/s^2]
        Kus;
        L;
        mu;
    end


    methods (Access = protected)
        function setupImpl(obj)
            modelParameters = evalin('base', 'model');

            obj.L = modelParameters.L;
            obj.mu = modelParameters.mu;
            obj.Kus = modelParameters.Kus;
        end

        function [Mz, r_error] = stepImpl(obj, Vx, delta, r)
            r_desired = obj.compute_r_desired(Vx, delta);
            r_error = r_desired - r;
            Mz = obj.Kp * r_error; 
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        function [Mz, r_error] = getOutputSizeImpl(~)
            % Return size for each output port
            Mz = [1 1];
            r_error = [1 1];
        end

        function [Mz, r_error] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            Mz = 'double';
            r_error = 'double';
        end

        function [Mz, r_error] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            Mz = false;
            r_error = false;
        end

        function [Mz, r_error] = isOutputFixedSizeImpl(~)
            % Return true if each output port has a fixed size
            Mz = true;
            r_error = true;
        end
    end

    methods (Access = private)
        function r_desired = compute_r_desired(obj, Vx, delta)
            % Avoid division by zero at standstill
            if Vx < 0.01
                r_desired = 0;
                return;
            end

            % --- 1. Calculate Driver's Intended Yaw Rate ---
            % This is the steady-state yaw rate response of the vehicle 
            % (Based on R047017_2025_lecture04.pdf, p. 15, 34)

            denominator = obj.L + (obj.Kus * Vx^2 / obj.gravity);

            if abs(denominator) < 0.01
                % Handle critical speed (oversteering) or invalid params
                r_unlimited = 0; 
            else
                % Calculate the unlimited yaw rate requested by the steering angle
                r_unlimited = (Vx / denominator) * delta;
            end

            % --- 2. Calculate Physical (Friction) Limit ---
            % The maximum possible yaw rate is limited by lateral friction.
            % a_y_max = mu * g
            % a_y_max = r * Vx  (at steady state)
            % So, r_max = (mu * g) / Vx
            % (Based on R047017_2025_lecture06.pdf, p. 25)
            r_limit = 0.85 * (obj.mu * obj.gravity) / Vx;

            % The final reference cannot ask for more than is physically possible.
            r_desired = max(-r_limit, min(r_limit, r_unlimited));

        end
    end
end
