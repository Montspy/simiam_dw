classdef GRITSBot < simiam.robot.Robot

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    
    properties
        wheel_radius		% m
        wheel_base_length	% m
        ticks_per_rev		% 1
        speed_factor
        
        encoders = simiam.robot.sensor.WheelEncoder.empty(1,0);
        ir_array = simiam.robot.sensor.ProximitySensor.empty(1,0);
		
		prev_ticks
		ir_coef
        
        dynamics
        
        firmware_3_0_plus
    end
    
    properties (SetAccess = private)
        right_wheel_speed
        left_wheel_speed
    end
    
    methods
        function obj = GRITSBot(parent, pose)
           obj = obj@simiam.robot.Robot(parent, pose);
           
           % Add surfaces: GRITSBot in top-down 2D view
           gb_top_plate =  [ 0.013  0.015 1;
                             0.013 -0.015 1;
                            -0.018 -0.015 1;
                            -0.018  0.015 1];
                          
           gb_base =  [  0.005  0.019 1;
                         0.005 -0.019 1;
                        -0.005 -0.019 1;
                        -0.005  0.019 1];
           
           
            obj.add_surface(gb_base, [ 0 0 0 ]/255);
            obj.add_surface(gb_top_plate, [ 51 0 85 ]/255);
            
            % Add sensors: wheel encoders and IR proximity sensors
            obj.wheel_radius = 0.005;           % 5mm
            obj.wheel_base_length = 0.035;      % 35mm
            
			
			obj.ticks_per_rev = 16;	% 16 tick <-> 1 revolution
			obj.speed_factor = 2*pi;	% vel [rad/s] = obj.speed_factor*n [rps]
            
            obj.encoders(1) = simiam.robot.sensor.WheelEncoder('right_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            obj.encoders(2) = simiam.robot.sensor.WheelEncoder('left_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
			obj.prev_ticks.left = 0;
			obj.prev_ticks.right = 0;
			
			% Obtained via curve fitting (cftool in Matlab). distance(raw) = (p1*raw + p2)/(raw + q1)
			obj.ir_coef = [	-30.30, 36320, -197.1;	% p1, p2, q1
							-8.406, 20000, -116.0;
							-36.05, 44120, -21.74;
							-49.16, 57820, -19.86;
							-22.39, 28450, -241.8;
							-34.29, 45600, -152.0];
            
            import simiam.robot.sensor.ProximitySensor;
            import simiam.robot.GRITSBot;
            import simiam.ui.Pose2D;
            
            noise_model = simiam.robot.sensor.noise.GaussianNoise(0,0);
            
            for i = 1:6
                ir_pose = Pose2D(-0.0025 + 0.015*cos((i-1)*pi/3), 0.015*sin((i-1)*pi/3), (i-1)*pi/3);
                obj.ir_array(i) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.01, 0.1, Pose2D.deg2rad(60), 'simiam.robot.GRITSBot.ir_distance_to_raw', noise_model);
            end
           
            % Add dynamics: two-wheel differential drive
            obj.dynamics = simiam.robot.dynamics.DifferentialDrive(obj.wheel_radius, obj.wheel_base_length);
            
            obj.right_wheel_speed = 0;
            obj.left_wheel_speed = 0;
        end
        
        function ir_distances = get_ir_distances(obj)
            ir_distances = obj.ir_array.get_range();
        end
        
        % Hardware connectivty related functions
        function add_hardware_link(obj, hostname, port)
            obj.driver = simiam.robot.driver.GritsBotDriver(hostname, port);
        end
        
        function pose_new = update_state_from_hardware(obj, pose, dt)
            
            encoder_ticks = obj.driver.get_encoder_ticks();
            
            if (~isempty(encoder_ticks))
                obj.encoders(2).ticks = encoder_ticks(1);
                obj.encoders(1).ticks = encoder_ticks(2);
            end
            
            ir_raw_values = obj.driver.get_ir_raw_values();
            
            if (~isempty(ir_raw_values))
                p1 = obj.ir_coef(:,1);
				p2 = obj.ir_coef(:,2);
				q1 = obj.ir_coef(:,3);
				ir_order = [1 6 5 4 3 2];
				
                for i = 1:numel(obj.ir_array)
					ir_distances(i) = (p1(i)*ir_raw_values(i) + p2(i))/(q1(i) + ir_raw_values(i));
                    obj.ir_array(ir_order(i)).update_range(ir_distances(i));
                end
            end
            
            obj.driver.set_speeds(obj.right_wheel_speed, obj.left_wheel_speed);
            
            pose_new = obj.update_pose_from_hardware(pose);
            
            obj.update_pose(pose_new);
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose_new);
            end
        end
        
        function pose_k_1 = update_pose_from_hardware(obj, pose)
            right_ticks = obj.encoders(1).ticks;
            left_ticks = obj.encoders(2).ticks;
            
            prev_right_ticks = obj.prev_ticks.right;
            prev_left_ticks = obj.prev_ticks.left;
            
            obj.prev_ticks.right = right_ticks;
            obj.prev_ticks.left = left_ticks;
            
            [x, y, theta] = pose.unpack();
                        
            m_per_tick = (2*pi*obj.wheel_radius)/obj.encoders(1).ticks_per_rev;
            
            d_right = (right_ticks-prev_right_ticks)*m_per_tick;
            d_left = (left_ticks-prev_left_ticks)*m_per_tick;
            
            d_center = (d_right + d_left)/2;
            phi = (d_right - d_left)/obj.wheel_base_length;
            
            theta_new = theta + phi;
            x_new = x + d_center*cos(theta);
            y_new = y + d_center*sin(theta);
                                       
            % Update your estimate of (x,y,theta)
            pose_k_1 = simiam.ui.Pose2D(x_new, y_new, theta_new);
        end
        
        
        function pose = update_state(obj, pose, dt)
            sf = obj.speed_factor;
            
            vel_r = obj.right_wheel_speed*sf;     % mm/s
            vel_l = obj.left_wheel_speed*sf;      % mm/s
            
            pose = obj.dynamics.apply_dynamics(pose, dt, vel_r, vel_l);
            obj.update_pose(pose);
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose);
            end
            
            % update wheel encoders
            
            obj.encoders(1).update_ticks(vel_r, dt);
            obj.encoders(2).update_ticks(vel_l, dt);
        end
        
        function set_wheel_speeds(obj, vel_r, vel_l)
            [vel_r, vel_l] = obj.limit_speeds(vel_r, vel_l);
            
            sf = obj.speed_factor;
            
            obj.right_wheel_speed = vel_r/sf;
            obj.left_wheel_speed = vel_l/sf;
        end
        
        function [vel_r, vel_l] = get_wheel_speeds(obj)
            sf = obj.speed_factor;
            
            vel_r = obj.right_wheel_speed*sf;
            vel_l = obj.left_wheel_speed*sf;
        end
        
        function [vel_r, vel_l] = limit_speeds(obj, vel_r, vel_l)
            % actuator hardware limits
            
            %[v,w] = obj.dynamics.diff_to_uni(vel_r, vel_l);
%             v = max(min(v,0.314),-0.3148);
%             w = max(min(w,2.276),-2.2763);
%             [vel_r, vel_l] = obj.dynamics.uni_to_diff(v,w);

            sf = obj.speed_factor;
            
            max_vel = 2*sf;	% Maximum 2 rps
            
            vel_r = max(min(vel_r, max_vel), -max_vel);
            vel_l = max(min(vel_l, max_vel), -max_vel);
        end
    end
    
    methods (Static)
        function raw = ir_distance_to_raw(varargin)
            distance = cell2mat(varargin);
			raw = distance;
        end
    end
    
end

