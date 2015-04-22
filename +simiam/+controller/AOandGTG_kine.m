classdef AOandGTG_kine < simiam.controller.Controller

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
% Dynamic Window style

    properties
        % plot support     
        p
        
        % Obstacle container
        obstacles           % 5*nObsMax matrix. Each column is two x,y points stacked (start and end point of the obstacle line) with a validity flag
        obsPointer
        obst_plots
        
        % sensor geometry
        calibrated
        sensor_placement
        
        step        % 1 to nDt
        finished
        oldInd
        lastNW
        
        state
        
        vRange
        wRange
        
        Vr
        dist
        
        x_g
        y_g
        
        va
        wa
        
        vComNew
        wComNew
        
        vCom
        wCom
    end
    
    properties (Constant)
        inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        outputs = struct('v', 0, 'w', 0)
        
        nObsMax = 6;       % 3 sensors, 2 samples kept
        sensorsInd = [6, 1, 2];
        
        vMax = pi/100;          % [m/s], 31.4 mm/s
        wMax = 50/180*pi;       % [rad/s], 50°/s
        vDotMax = .5;           % m/s^2
        wDotMax = 720/180*pi;   % rad/s^2
        
        nVSteps = 11;       % Discretize Vd into nVSteps segments along the v axis
        nWSteps = 51;       % Discretize Vd into nWSteps segments along the w axis
        
        coef_h = .1;
        coef_d = 4;
        coef_v = .1;
        
        nDt = 1;            % Time interval between commands (nDt*dt)
        nDtHorizon = 20;    % Time horizon (nDtHorizon*dt)
    end
    
    methods
        
        function obj = AOandGTG_kine()
            obj = obj@simiam.controller.Controller('ao_and_gtg_kine');            
            obj.calibrated = false;
            
%             obj.p = simiam.util.Plotter();

            obj.obstacles = zeros(5, obj.nObsMax);
            obj.obsPointer = 1;
            obj.obst_plots = simiam.containers.ArrayList(obj.nObsMax+obj.nWSteps);
            
            obj.step = 1;
            obj.vCom = 0;
            obj.wCom = 0;
            
            obj.lastNW = floor(obj.nWSteps/2);
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            tic;
            % Compute the placement of the sensors
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
                
                % Temporoary plot support
                hold(robot.parent, 'on');
%                 obj.u_arrow_r = plot(robot.parent, [0 0], [0 0], 'b-x', 'LineWidth', 2);
%                 obj.u_arrow = plot(robot.parent, [0 0], [0 0], 'r--x', 'LineWidth', 2);
%                 obj.s_net = plot(robot.parent, zeros(1,9), zeros(1,9), 'kx', 'MarkerSize', 8);
%                 set(obj.u_arrow_r, 'ZData', ones(1,2));
%                 set(obj.u_arrow, 'ZData', ones(1,2));
%                 set(obj.s_net, 'ZData', ones(1,9));
                
                for n=1:obj.nObsMax
                    plot_elem = plot(robot.parent, [0 0], [0 0], 'k-', 'LineWidth', 4);
                    set(plot_elem, 'ZData', ones(1,2));
                    obj.obst_plots.appendElement(plot_elem);
                end
                
                for n=1:obj.nWSteps
                    plot_elem = plot(robot.parent, [0 0], [0 0], 'b-', 'LineWidth', 1);
                    set(plot_elem, 'ZData', ones(1,2));
                    obj.obst_plots.appendElement(plot_elem);
                end
            end
            
            if(obj.step == 1) 
                obj.state = state_estimate;
                
                % Get goal position
                obj.x_g = inputs.x_g;
                obj.y_g = inputs.y_g;
                
                % Get the current curvature
                [vel_r, vel_l] = robot.get_wheel_speeds();
                [obj.va, obj.wa] = robot.dynamics.diff_to_uni(vel_r, vel_l);
%                 obj.va = 0.4;   % 40cm/s
%                 obj.wa = 0;
                
                % Poll the current IR sensor values 1-9
                ir_distances = robot.get_ir_distances();

                % Interpret the IR sensor measurements geometrically
                ir_distances_wf = obj.apply_sensor_geometry(ir_distances);    

                % 1. Update the obstacle line list
                obj.update_obstacle_line_list(robot, ir_distances, ir_distances_wf);

                % 2. Dynamic window (Vd)
                obj.generate_dynamic_window_matrix(obj.nDtHorizon*dt);

                % 3. Attainable curvatures (Vs)
                obj.mark_unattainable_curvatures(robot);
                
                obj.finished = false;
                obj.dist = zeros(size(obj.Vr));
                obj.oldInd = 1;
            end

            
            if(~obj.finished)
                % 4. Test for admissibility (Va)
                obj.compute_admissibility(robot, dt);
            
                if(obj.finished) 
                    % 5. Compute objective function G(v,w) over Vr and find maximum
                    [v, w] = obj.maximize_objective_function(obj.nDtHorizon*dt);
                    
                    %disp('Finished');
                    %disp(obj.step);
                    toc;

                    obj.vComNew = v;
                    obj.wComNew = w;
                else
                    disp(obj.oldInd);
                end
            end
                
            obj.step = obj.step + 1;
            if(obj.step > obj.nDt)
                obj.step = 1;
                
                obj.vCom = obj.vComNew;
                obj.wCom = obj.wComNew;
            end
            
            outputs.v = obj.vCom;
            outputs.w = obj.wCom;
        end
        
        % Helper functions
        
        function ir_distances_wf = apply_sensor_geometry(obj, ir_distances)
                    
            % 1. Apply the transformation to robot frame.
            nSensors = numel(ir_distances);
            
            ir_distances_rf = zeros(3,nSensors);
            for i=1:nSensors
                x_s = obj.sensor_placement(1,i);
                y_s = obj.sensor_placement(2,i);
                theta_s = obj.sensor_placement(3,i);
                
                R = obj.get_transformation_matrix(x_s,y_s,theta_s);
                ir_distances_rf(:,i) = R*[ir_distances(i); 0; 1];
            end
            
            % 2. Apply the transformation to world frame.
            
            [x,y,theta] = obj.state.unpack();
            
            R = obj.get_transformation_matrix(x,y,theta);
            ir_distances_wf = R*ir_distances_rf;
            
            ir_distances_wf = ir_distances_wf(1:2,:);
        end
        
        function update_obstacle_line_list(obj, robot, ir_distances, ir_distances_wf)
            [x, y, ~] = obj.state.unpack();
            
            % For all sensors
            for n=obj.sensorsInd
                angle = robot.ir_array(n).spread;
                max_range = robot.ir_array(n).max_range;
                spreading_multiplier = 2*tan(angle/2) * 1.5;
                
                % Find a normal to the main sensor axis
                normal = [(ir_distances_wf(2,n)-y) ; -(ir_distances_wf(1,n)-x)];
                normal = normal/norm(normal);
                
                % Creat the obstacle line/segment
                length = max(ir_distances(n) * spreading_multiplier, max_range*3/4*spreading_multiplier);
                a = ir_distances_wf(:,n) + normal*length/2;
                b = ir_distances_wf(:,n) - normal*length/2;
                
                % Append it to the list
                obj.obsPointer = obj.obsPointer+1;
                if(obj.obsPointer > obj.nObsMax)
                    obj.obsPointer = 1;
                end
                
                obj.obstacles(:, obj.obsPointer) = [a; b; 1];
            end
            
            for n=1:obj.nObsMax
                if(obj.obstacles(5, n) == 0)
                    continue;
                end
                
                S = obj.obstacles(1:4, n);
                
                % Plot all
                set(obj.obst_plots.elementAt(n), 'XData', [S(1), S(3)]);
                set(obj.obst_plots.elementAt(n), 'YData', [S(2), S(4)]);
            end
            
        end
        
        function generate_dynamic_window_matrix(obj, dt)
            obj.Vr = ones(obj.nVSteps, obj.nWSteps);
            
%             obj.vRange = [max(0, obj.va - obj.vDotMax*dt),         min(obj.vMax, obj.va + obj.vDotMax*dt)];
%             obj.wRange = [max(-obj.wMax, obj.wa - obj.wDotMax*dt), min(obj.wMax, obj.wa + obj.wDotMax*dt)];
%             
            obj.vRange = [0, obj.vMax];
            obj.wRange = [-obj.wMax, obj.wMax];
            
%             for nV=1:obj.nVSteps
%                 v_i = obj.vRange(1) + (obj.vRange(2)-obj.vRange(1)) * nV/obj.nVSteps;
%                 for nW=1:obj.nWSteps
%                     w_i = obj.wRange(1) + (obj.wRange(2)-obj.wRange(1)) * nW/obj.nWSteps;
%                     
%                     if(w_i >= min(obj.wMax, obj.wa + obj.wDotMax*dt))
%                         obj.Vr(nV, nW) = 0;
%                     elseif(w_i <= max(-obj.wMax, obj.wa - obj.wDotMax*dt))
%                         obj.Vr(nV, nW) = 0;
%                     end
%                     
%                     if(v_i >= min(obj.vMax, obj.va + obj.vDotMax*dt))
%                         obj.Vr(nV, nW) = 0;
%                     elseif(v_i <= max(0, obj.va - obj.vDotMax*dt))
%                         obj.Vr(nV, nW) = 0;
%                     end
%                 end
%             end
        end
        
        function mark_unattainable_curvatures(obj, robot)
            % Test all curvatures (v_i, w_i) in Vd
%             for nV=1:obj.nVSteps
%                 v_i = obj.vRange(1) + (obj.vRange(2)-obj.vRange(1)) * nV/obj.nVSteps;
%                 for nW=1:obj.nWSteps
%                     w_i = obj.wRange(1) + (obj.wRange(2)-obj.wRange(1)) * nW/obj.nWSteps;
%                     
%                     [vel_r, vel_l] = robot.dynamics.uni_to_diff(v_i, w_i);
%                     
%                     if ((abs(vel_r) > obj.vMax/robot.wheel_radius) || ...
%                             (abs(vel_l) > obj.vMax/robot.wheel_radius) || ...
%                             v_i < 0)   % Impossible wheel speed
%                         obj.Vr(nV, nW) = 0;
%                     end
%                 end
%             end
            
        end
        
        function compute_admissibility(obj, robot, loopTime)
            obj.finished = true;
            
            [x, y, theta] = obj.state.unpack();
            theta = atan2(sin(theta), cos(theta));
            
            % For each curvature (v_i, w_i)
%             for nV=1:obj.nVSteps
%                 v_i = obj.vRange(1) + (obj.vRange(2)-obj.vRange(1)) * nV/obj.nVSteps;

                theta_g = atan2(obj.y_g-y, obj.x_g-x);
                theta_pi_pi = atan2(sin(theta), cos(theta));    % -pi <= theta_pi_pi < pi
                delta_theta = atan2(sin(theta_g - theta_pi_pi), cos(theta_g - theta_pi_pi));    % -pi <= delta_theta < pi
                
                bestNW = round((delta_theta/(obj.nDtHorizon*loopTime) - obj.wRange(1))*obj.nWSteps/(obj.wRange(2) - obj.wRange(1)));    % Toward the goal next timestep
                if(bestNW > obj.nWSteps)
                    bestNW = obj.nWSteps;
                elseif(bestNW < 1)
                    bestNW = 1;
                end
                
                upperLimit = 0;
                lowerLimit = 0;
                for n=1:obj.nWSteps
                    if(n < obj.oldInd)
                        continue;   % Skip this loop if already checked in previous steps
                    end
                    
                    if(~upperLimit && ~lowerLimit)  % Convert 1, 2, 3, 4, 5, ... to best, best+1, best-1, best+2, best-2, ...
                        if(mod(n,2) == 0)   % Even
                            nW = bestNW + floor(n/2);
                        else    % Odd
                            nW = bestNW - floor(n/2);
                        end
                    elseif(upperLimit)
                        nW = obj.nWSteps - n + 1;
                    elseif(lowerLimit)
                        nW = n;
                    end
                    
                    if(nW == 1)   lowerLimit = 1; end
                    if(nW == obj.nWSteps)   upperLimit = 1; end
                    
                    w_i = obj.wRange(1) + (obj.wRange(2)-obj.wRange(1)) * nW/obj.nWSteps;
                    
                    % Test admissibility of curvature (v_i, w_i)
%                         stopDist = 0;
%                         temp_v = v_i;
%                         while(temp_v > 0)
%                             stopDist = stopDist + temp_v*obj.nDt*loopTime;
%                             temp_v = temp_v - obj.vDotMax*obj.nDt*loopTime;
%                         end
                    minDist_admi = 0.05;
                    maxDist = .1;
                    minDist = maxDist;

                    theta_traj = theta + w_i*obj.nDtHorizon*obj.nDt*loopTime;

                    % Slope and y-intercept of trajectory
                    s = tan(theta_traj);
                    yint = y - s*x;
                    % Velocity vector
                    v = [cos(theta_traj); sin(theta_traj)];

                    for nO=1:obj.nObsMax   % Test for collision with every obstacle line
                        if(obj.obstacles(5, nO) == 0)
                            continue;
                        end

                        A = obj.obstacles(1:2, nO);
                        B = obj.obstacles(3:4, nO);

                        if(B(1) < A(1)) % A is always to the left of B, swap if not
                            tmp = B;
                            B = A;
                            A = tmp;
                        end

                        % Line slope and y-intersect of obstacle line
                        sO = (B(2)-A(2))/(B(1)-A(1));
                        yintO = A(2) - sO*A(1);

                        % Find intersection between trajectory and (AB)
                        xout = (yint - yintO)/(sO - s);
                        yout = s*xout + yint;

                        % Is tajectory intersecting with segment [AB]?
                        if((A(1) <= xout) && (B(1) >= xout))
                            delta = [xout-x ; yout-y];

                            if(dot(delta, v) > 0)   % Obstacle is in front of the robot
                                minDist_r = norm(delta);

                                if(minDist_r < minDist)
                                    minDist = minDist_r;
                                end
                            end
                        end
                    end
                    
                    for nV=1:obj.nVSteps
                        obj.dist(nV,nW) = minDist;

                        if(minDist < minDist_admi*(nV/obj.nVSteps))  % Curvature not admissible <=> not enough distance to stop before obstacle
                            obj.Vr(nV,nW) = 0;
                            obj.dist(nV,nW) = 0;
                        end
                    end

                    set(obj.obst_plots.elementAt(obj.nObsMax+nW), 'XData', [x, x+cos(theta_traj)*minDist]);
                    set(obj.obst_plots.elementAt(obj.nObsMax+nW), 'YData', [y, y+sin(theta_traj)*minDist]);
                    
                    time = toc;
                    if((time >= 0.90*loopTime) && (obj.step < obj.nDt))    % Stop loop if loopTime is 90% spent and no the last step
                        obj.finished = false;
                        obj.oldInd = n;
                        break;
                    end
                end
%             end
        end
        
        function [v, w] = maximize_objective_function(obj, dt)
            H = zeros(size(obj.Vr));
            V = zeros(size(obj.Vr));
            
            % For each curvature (v_i, w_i)
            for nV=1:obj.nVSteps
                v_i = obj.vRange(1) + (obj.vRange(2)-obj.vRange(1)) * nV/obj.nVSteps;
                for nW=1:obj.nWSteps
                    w_i = obj.wRange(1) + (obj.wRange(2)-obj.wRange(1)) * nW/obj.nWSteps;
                    
                    if obj.Vr(nV, nW) == 1
                        % Heading
                        [x, y, theta] = obj.state.unpack();
                        
                            % 1st interval: curvature (v_i, w_i)
                        if w_i ~= 0
                            x = x + v_i/w_i*(sin(theta)-sin(theta+w_i*dt));
                            y = y - v_i/w_i*(cos(theta)-cos(theta+w_i*dt));
                            theta = theta + w_i*dt;
                        else    % w_i == 0
                            x = x + v_i*cos(theta)*dt;
                            y = y + v_i*sin(theta)*dt;
                        end
                        
                        theta_g = atan2(obj.y_g-y, obj.x_g-x);
                        theta_pi_pi = atan2(sin(theta), cos(theta));    % -pi <= theta_pi_pi < pi
                        delta_theta = atan2(sin(theta_g - theta_pi_pi), cos(theta_g - theta_pi_pi));    % -pi <= delta_theta < pi
                        
                        H(nV, nW) = 180 - 180/pi*abs(delta_theta);
                        
                        % Velocity
                        V(nV, nW) = max(v_i, 0);
                    end
                end
            end
            
            % Normalize H
            M = 180;    % Max
            m = 0;      % Min
%             M = max(H(obj.Vr == 1));    % Use only indices where Vr == 1
%             m = min(H(obj.Vr == 1));
            if(M ~= m)
                H = bsxfun(@times, bsxfun(@minus, H, m), 1/(M-m)*obj.Vr);
            else
                H = zeros(size(obj.Vr));
            end
            
            % Normalize dist
            M = .1;     % Max
            m = 0;      % Min
%             M = max(obj.dist(obj.Vr == 1));
%             m = min(obj.dist(obj.Vr == 1));
            if(M ~= m)
                obj.dist = bsxfun(@times, bsxfun(@minus, obj.dist, m), 1/(M-m)*obj.Vr);
            else
                obj.dist = zeros(size(obj.Vr));
            end
            
            % Normalize V
            M = obj.vMax;   % Max
            m = 0;          % Min
%             M = max(V(obj.Vr == 1));
%             m = min(V(obj.Vr == 1));
            if(M ~= m)
                V = bsxfun(@times, bsxfun(@minus, V, m), 1/(M-m)*obj.Vr);
            else
                V = zeros(size(obj.Vr));
            end
                        
            G = obj.coef_h*H + obj.coef_d*obj.dist + obj.coef_v*V;
            G(obj.Vr == 0) = 0;   % Remove invalid indices
            
%             figure(2);
%             mesh(linspace(obj.wRange(1)*180/pi, obj.wRange(2)*180/pi, obj.nWSteps), ...
%                  linspace(obj.vRange(1), obj.vRange(2), obj.nVSteps), ...
%                  H);
% 
%             xlabel('W');
%             ylabel('V');
%             title('Heading');
            
%             figure(3);
%             mesh(linspace(obj.wRange(1)*180/pi, obj.wRange(2)*180/pi, obj.nWSteps), ...
%                  linspace(obj.vRange(1), obj.vRange(2), obj.nVSteps), ...
%                  obj.dist);
% 
%             xlabel('W');
%             ylabel('V');
%             title('Distance');
            
%             figure(4);
%             mesh(linspace(obj.wRange(1)*180/pi, obj.wRange(2)*180/pi, obj.nWSteps), ...
%                  linspace(obj.vRange(1), obj.vRange(2), obj.nVSteps), ...
%                  V);
% 
%             xlabel('W');
%             ylabel('V');
%             title('Velocity');
            
%             figure(5);
%             mesh(linspace(obj.wRange(1)*180/pi, obj.wRange(2)*180/pi, obj.nWSteps), ...
%                  linspace(obj.vRange(1), obj.vRange(2), obj.nVSteps), ...
%                  G);
% 
%             xlabel('W');
%             ylabel('V');
%             title('Objective function');

            [~, ind] = max(G(:));
            [nV, nW] = ind2sub(size(G),ind);
            
            if(G(nV, nW) == 0)    % No valid maximum
                v = max(min(0, obj.vRange(2)), obj.vRange(1));  % Brake
                w = max(min(0, obj.wRange(2)), obj.wRange(1));
                obj.lastNW = floor(obj.nWSteps/2);
            else
                v = obj.vRange(1) + (obj.vRange(2)-obj.vRange(1)) * nV/obj.nVSteps;
                w = obj.wRange(1) + (obj.wRange(2)-obj.wRange(1)) * nW/obj.nWSteps;
                obj.lastNW = nW;
            end
        end
        
        function set_sensor_geometry(obj, robot)
            nSensors = numel(robot.ir_array);
            
            obj.sensor_placement = zeros(3,nSensors);
            for i=1:nSensors
                [x, y, theta] = robot.ir_array(i).location.unpack();
                obj.sensor_placement(:,i) = [x; y; theta];
            end                        
            obj.calibrated = true;
        end
        
        function R = get_transformation_matrix(obj, x, y, theta)
            R = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
        end
        
        function reset(obj)
            % Reset accumulated and previous error
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
    end
    
end

