classdef pathTracking_CarTrailer < handle
    
    % Path Tracking for Car Trailer System
    %
    % Author: Junyu Zhou
    %
    % Reference: Reversing the General One-Trailer System: Asymptotic
    % Curvature Stabilization and Path Tracking
    %
    % Copyright (c) 2020, Junyu Zhou
    % All rights reserved.
    % License : Modified BSD Software License Agreement
    
    properties
        % parameters of car and trailer
        CarLength
        CarWidth
        CarHitch
        WheelBase
        WheelTrack
        TrailerBase
        TrailerTrack
        
        % position and rotation of car and trailer
        Xcar
        Ycar
        Phicar
        Phitrailer
        Steering
        Velociy
        
        % path tracking parameters
        K1
        K2
        K3
        
%         % map parameters
%         MapDimX
%         MapDimY
%         StartPoint
%         GoalPoint
%         Obstacle
%         
%         % resolution of A* algorithm
%         TickXY
%         TickAngle
%         
%         % calculated path
%         Path
%         SteeringWheel
        
    end
    
    properties (Access = private)
        AniFig
    end
    
    methods
        
        function obj = pathTracking_CarTrailer(CarLength, CarWidth, WheelBase, ...
                WheelTrack, TrailerBase, TrailerTrack, CarHitch, Xcar,...
                Ycar, Phicar, Phitrailer, Steering, Velocity)
            if nargin == 13
                obj.CarLength = CarLength;
                obj.CarWidth = CarWidth;
                obj.CarHitch = CarHitch;
                obj.WheelBase = WheelBase;
                obj.WheelTrack = WheelTrack;
                obj.TrailerBase = TrailerBase;
                obj.TrailerTrack = TrailerTrack;
                obj.Xcar = Xcar;
                obj.Ycar = Ycar;
                obj.Phicar = Phicar;
                obj.Phitrailer = Phitrailer;
                obj.Steering = Steering;
                obj.Velociy = Velocity;
            else
                obj.CarLength = 3.5;
                obj.CarWidth = 2;
                obj.CarHitch = 0.3;
                obj.WheelBase = 2.5;
                obj.WheelTrack = 1.6;
                obj.TrailerBase = 2;
                obj.TrailerTrack = 1.5;
                obj.Xcar = 2.3;
                obj.Ycar = 30;
                obj.Phicar = 0;
                obj.Phitrailer = 0;
                obj.Steering = 0;
                obj.Velociy = -1;
            end
            
            obj.K1 = 2;
            obj.K2 = 20;
            obj.K3 = 0;
            obj.AniFig = figure; %[];
            
        end
        
        function testCarTrailer(obj)
            dt = 0.1;
            for i = 1 : 10
                moveStep(obj, dt);
            end
        end
        
        function testPathTracking_Circle(obj)
            
            h = obj.AniFig;
            figure(h)
            hold on
            set(gcf,'position',[200,100,800,750])
            set(gcf,'color','w');
            axis([-30 10 15 35])
            xlabel('[m]')
            ylabel('[m]')
            obj.drawCar(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Steering(end));
            obj.drawTrailer(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Phitrailer(end));
            box on
            grid on
            r_d = 30;
            
            error = [0 0];
            SW_Ang = [0 0];
            dt = 0.1;
            for i = 1 : 300
                r_d = 30;
                [xt, yt] = obj.calcTrailerPos();
                r = sqrt(xt^2 + yt^2);
                d = r - r_d;
                error = [error; [error(end,1)+dt, d]]; 
                if abs(d) > 0.3
                    d = sign(d)*0.3;
                end
                r_d = r - d;
                Kp_d = 1/ r_d;
                Kp1_d = 0;
                phi_d = atan(-xt/yt);
                
                V =obj.calcV(Kp_d, phi_d, d);
                
                obj.pathTracking_steering(Kp1_d, V)
                if mod(i, 50) == 0
                    obj.moveStep(dt, true);
                else
                    obj.moveStep(dt, false);
                end
                plot(-r_d*sin(phi_d), r_d*cos(phi_d), 'ro')
                SW_Ang = [SW_Ang; [SW_Ang(end,1)+dt, obj.Steering(end)]];

            end
            obj.drawCar(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Steering(end));
            obj.drawTrailer(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Phitrailer(end));
            
            error(1,:) = [];
            figure;
            subplot(2,1,1)
            plot(error(:,1), error(:,2))
            xlabel('time [s]')
            ylabel('deviation [m]')
            box on
            grid on
            axis([0 error(end,1) min(error(:,2)) max(error(:,2))])
            
            subplot(2,1,2)
            plot(SW_Ang(:,1), SW_Ang(:,2)*180/pi)
            xlabel('time [s]')
            ylabel('steering angle [°]')
            box on
            grid on
            set(gcf,'position',[200,100,800,300])
            axis([0 SW_Ang(end,1) min(SW_Ang(:,2)*180/pi) max(SW_Ang(:,2)*180/pi)])
        
        end
        
        function testPathTracking_StraightLine(obj)
            h = obj.AniFig;
            figure(h)
            set(gcf,'position',[200,100,800,150])
            set(gcf,'color','w');
            axis([-40 10 28 32])
            xlabel('[m]')
            ylabel('[m]')
            hold on
            obj.drawCar(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Steering(end));
            obj.drawTrailer(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Phitrailer(end));
            box on
            grid on
            
            error = [0 0];
            SW_Ang = [0 0];
            dt = 0.1;
            for i = 1 : 300
                
                [xt, yt] = obj.calcTrailerPos();
                d = yt - 30;
                error = [error; [error(end,1)+dt, d]];
                if abs(d) > 0.3
                    d = sign(d)*0.3;
                end
                Kp_d = 0;
                Kp1_d = 0;
                phi_d = 0;
                
                V =obj.calcV(Kp_d, phi_d, d);
                
                obj.pathTracking_steering(Kp1_d, V)
                if mod(i, 50) == 0
                    obj.moveStep(dt, true);
                else
                    obj.moveStep(dt, false);
                end
                plot(xt, 30, 'ro')
                SW_Ang = [SW_Ang; [SW_Ang(end,1)+dt, obj.Steering]];
                
            end
            
            obj.drawCar(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Steering(end));
            obj.drawTrailer(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Phitrailer(end));
            
            error(1,:) = [];
            figure;
            subplot(2,1,1)
            plot(error(:,1), error(:,2))
            xlabel('time [s]')
            ylabel('deviation [m]')
            box on
            grid on
            axis([0 error(end,1) min(error(:,2)) max(error(:,2))])
            
            subplot(2,1,2)
            plot(SW_Ang(:,1), SW_Ang(:,2)*180/pi)
            xlabel('time [s]')
            ylabel('steering angle [°]')
            box on
            grid on
            set(gcf,'position',[200,100,800,300])
            axis([0 SW_Ang(end,1) min(SW_Ang(:,2)*180/pi) max(SW_Ang(:,2)*180/pi)])
        
        end
        
%         function testPathTracking_SloptraightLine(obj)
%             h = obj.AniFig;
%             hold on
%             obj.drawCar(h, obj.Xcar, obj.Ycar, obj.Phicar, obj.Steering);
%             obj.drawTrailer(h, obj.Xcar, obj.Ycar, obj.Phicar, obj.Phitrailer);
%             box on
%             grid on
%             
%             SlopAngle = 10/180*pi;
%             for i = 1 : 300
%                 [xt, yt] = obj.calcTrailerPos();
%                 yt = yt - 30;
%                 d = -xt*sin(SlopAngle) + yt*cos(SlopAngle);
%                 d
%                 x_coor = xt*cos(SlopAngle) + yt*sin(SlopAngle);
%                 if abs(d) > 0.3
%                     d = sign(d)*0.3;
%                 end
%                 
%                 plot(x_coor*cos(SlopAngle), x_coor*sin(SlopAngle)+30, 'ro')
%                 
%                 Kp_d = 0;
%                 Kp1_d = 0;
%                 phi_d = SlopAngle;
%                 dt = 0.1;
%                 
%                 V =obj.calcV(Kp_d, phi_d, d);
%                 
%                 obj.pathTracking_steering(Kp1_d, V)
%                 if mod(i, 50) == 0
%                     obj.moveStep(dt, true);
%                 else
%                     obj.moveStep(dt, false);
%                 end
%                 
%             end
%             
%             obj.drawCar(h, obj.Xcar, obj.Ycar, obj.Phicar, obj.Steering);
%             obj.drawTrailer(h, obj.Xcar, obj.Ycar, obj.Phicar, obj.Phitrailer);
%         end

        function animationCircle(obj)
            ang = linspace(0,pi/2/1.5,100);
            r_d = 30;
            x = -r_d*sin(ang);
            y = r_d*cos(ang);
            Path = [x' y'];
            
            animation(obj, Path, 'CircleTracking');
            
        end
    end
    
 
    
    methods (Access = private)
        
        function animation(obj, Path, VideoName)
            
            h = figure;
            xmin = min(Path(:,1))-3;
            xmax = max(Path(:,1))+5;
            ymin = min(Path(:,2))-3;
            ymax = max(Path(:,2))+3;
            
            set(gcf,'position',[200,100,800,400])
            set(gcf,'color','w');
            filename = [VideoName, '.gif'];
                        
            for i = 1 : length(obj.Xcar)
                
                plot(Path(:,1), Path(:,2), 'r');
                hold on
                obj.drawCar(h, obj.Xcar(i), obj.Ycar(i), obj.Phicar(i), obj.Steering(i));
                obj.drawTrailer(h, obj.Xcar(i), obj.Ycar(i), obj.Phicar(i), obj.Phitrailer(i));
                drawnow
                hold off
                axis([xmin, xmax, ymin, ymax])
                xlabel('[m]')
                ylabel('[m]')
                box on
                grid on
                frame = getframe(h);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                if i == 1
                    imwrite(imind,cm,filename,'gif','DelayTime',0, 'Loopcount',inf);
                else
                    imwrite(imind,cm,filename,'gif','DelayTime',0, 'WriteMode','append');
                end
            end
            
        end
        
        function pathTracking_steering(obj, Kp1_d, V)
            lt = obj.TrailerBase;
            lv = obj.WheelBase;
            lc = obj.CarHitch;
            T = 0.2;
            v = obj.Velociy;
            theta = obj.Steering(end);
            phiv = obj.Phicar(end);
            phit = obj.Phitrailer(end);
            
            u = lt*T*v*cos(theta)*cos(theta)/lv^2/lc*...
                ((lv*cos(phiv-phit)+lc*sin(phiv-phit)*tan(theta))^3*...
                (Kp1_d + V) - (lv^2 + lc^2*tan(theta)*tan(theta))/lt^2*...
                (-lt*tan(theta) + lv*sin(phiv-phit) - lc*cos(phiv-phit)*tan(theta)))+...
                theta;
            
            if abs(u) > 30/180*pi
                obj.Steering = [obj.Steering, sign(u) * 30/180*pi];
            else
                obj.Steering = [obj.Steering, u];
            end
        end
        
        function h1 = hd1(obj, phi_d)
            phit = obj.Phitrailer(end);
            h1 = phi_d - phit;
        end
        
        function h2 = hd2(obj, Kp_d)
            lt = obj.TrailerBase;
            lv = obj.WheelBase;
            lc = obj.CarHitch;
            theta = obj.Steering(end);
            phiv = obj.Phicar(end);
            phit = obj.Phitrailer(end);
            
            h2 = -Kp_d + ...
                (lv*sin(phiv-phit)-lv*cos(phiv-phit)*tan(theta))/...
                lt/(lv*cos(phiv-phit)+lc*sin(phiv-phit)*tan(theta));
        end
        
        function V =calcV(obj, Kp_d, phi_d, d)
            e1 = d;
            e2 = obj.hd1(phi_d);
            e3 = obj.hd2(Kp_d);
            
            V = -obj.K1*e1 - obj.K2*e2 -obj.K3*e3;
        end
        
        function [xt, yt] = calcTrailerPos(obj)
            x = obj.Xcar(end);
            y = obj.Ycar(end);
            phicar = obj.Phicar(end);
            phitrailer = obj.Phitrailer(end);
            
            Trailerbase = obj.TrailerBase;
            Hitchdis = obj.CarHitch;
            
            x = x - Hitchdis*cos(phicar);
            y = y - Hitchdis*sin(phicar);
            
            
            xt = x - Trailerbase*cos(phitrailer);
            yt = y - Trailerbase*sin(phitrailer);
        end
        
        function drawTrailer(obj, h, x, y, theta_car, theta_trailer)
            
            carhitchdis = obj.CarHitch;
            x = x - carhitchdis*cos(theta_car);
            y = y - carhitchdis*sin(theta_car);
            
            trailerwidth = obj.TrailerTrack;
            trailerlength = obj.TrailerBase;
            
            rear = linspace(-trailerwidth/2, trailerwidth/2, 50);
            rear = [-trailerlength*ones(length(rear),1), rear'];
            
            rod = linspace(0, -trailerlength, 50);
            rod = [ rod', 0*ones(length(rod),1),];
            
            rod = obj.changePos(x, y, theta_trailer, rod);
            rear = obj.changePos(x, y, theta_trailer, rear);
            
            
            wl = linspace(-trailerlength-0.2, -trailerlength+0.2, 20);
            wl = [wl', trailerwidth/2*ones(length(wl),1)];
            
            wr = linspace(-trailerlength-0.2, -trailerlength+0.2, 20);
            wr = [wr', -trailerwidth/2*ones(length(wr),1)];
            
            wl = obj.changePos(x, y, theta_trailer, wl);
            wr = obj.changePos(x, y, theta_trailer, wr);
            
            
            figure(h)
            hold on
            plot(rear(:,1), rear(:,2), 'LineWidth',2, 'color', 'b')
            plot(rod(:,1), rod(:,2), 'LineWidth',2, 'color', 'b')
            plot(wl(:,1), wl(:,2), 'LineWidth',5, 'color', 'b')
            plot(wr(:,1), wr(:,2), 'LineWidth',5, 'color', 'b')
        end
        
        function drawCar(obj, h, x, y, theta, steering)
            
            carwidth = obj.CarWidth;
            carlength = obj.CarLength;
            wheelbase = obj.WheelBase;
            wheeltrack = obj.WheelTrack;
            carhitchdis = obj.CarHitch;
            
            rear = linspace(-carwidth/2, carwidth/2, 50);
            rear = [-carhitchdis*ones(length(rear),1), rear'];
            
            front = linspace(-carwidth/2, carwidth/2, 50);
            front = [(carlength-carhitchdis)*ones(length(front),1), front'];
            
            left = linspace(-carhitchdis, (carlength-carhitchdis), 50);
            left = [ left', carwidth/2*ones(length(left),1),];
            
            right = linspace(-carhitchdis, (carlength-carhitchdis), 50);
            right = [ right', -carwidth/2*ones(length(right),1),];
            
            left = obj.changePos(x, y, theta, left);
            right = obj.changePos(x, y, theta, right);
            front = obj.changePos(x, y, theta, front);
            rear = obj.changePos(x, y, theta, rear);
            
            
            wrl = linspace(-0.2, +0.2, 20);
            wrl = [wrl', wheeltrack/2*ones(length(wrl),1)];
            
            wrr = linspace(-0.2, +0.2, 20);
            wrr = [wrr', -wheeltrack/2*ones(length(wrr),1)];
            
            wrl = obj.changePos(x, y, theta, wrl);
            wrr = obj.changePos(x, y, theta, wrr);
            
            
            wfl = linspace(-0.2, +0.2, 20);
            wfl = [wfl', 0*ones(length(wfl),1)];
            %wfl = [wfl', carwidth/2*ones(length(wfl),1)-0.2];
            
            wfr = linspace(-0.2,+0.2, 20);
            wfr = [wfr', 0*ones(length(wfr),1)];
            %wfr = [wfr', -carwidth/2*ones(length(wfr),1)+0.2];
            
            wfl = obj.changePos(wheelbase, wheeltrack/2, steering, wfl);
            wfr = obj.changePos(wheelbase, -wheeltrack/2, steering, wfr);
            
            wfl = obj.changePos(x, y, theta, wfl);
            wfr = obj.changePos(x, y, theta, wfr);
            
            
            figure(h)
            hold on
            plot(rear(:,1), rear(:,2), 'LineWidth',2, 'color', 'k')
            plot(front(:,1), front(:,2), 'LineWidth',2, 'color', 'k')
            plot(left(:,1), left(:,2), 'LineWidth',2, 'color', 'k')
            plot(right(:,1), right(:,2), 'LineWidth',2, 'color', 'k')
            
            plot(wfl(:,1), wfl(:,2), 'LineWidth',5, 'color', 'r')
            plot(wfr(:,1), wfr(:,2), 'LineWidth',5, 'color', 'r')
            plot(wrl(:,1), wrl(:,2), 'LineWidth',5, 'color', 'r')
            plot(wrr(:,1), wrr(:,2), 'LineWidth',5, 'color', 'r')
        end        
        
        function stepAnimation(obj, node, steering)
            if isempty(obj.AniFig)
                obj.AniFig = figure;
                set(gcf,'color','w');
                obstacle = obj.Obstacle;
                DimX = obj.MapDimX;
                DimY = obj.MapDimY;
                hold off;
            
                if length(obstacle)>=1
                    plot(obstacle(:,1),obstacle(:,2),'ok');hold on;
                end
                
                axis([0-0.5 DimX+0.5 0-0.5 DimY+0.5])
                xlabel('[m]')
                ylabel('[m]')
                grid on;
                
            end
            h = obj.AniFig;
            
            obj.drawCar(h, node(1), node(2), node(3), steering);
            obj.drawTrailer(h, node(1), node(2), node(3), node(4));
        end
        
        function moveStep(obj, dt, DrawCar)
            x = obj.Xcar(end);
            y = obj.Ycar(end);
            phicar = obj.Phicar(end);
            phitrailer = obj.Phitrailer(end);
            
            Wheelbase = obj.WheelBase;
            Trailerbase = obj.TrailerBase;
            Hitchdis = obj.CarHitch;
            
            SteeringCar = obj.Steering(end);
            Velocity = obj.Velociy;
            
            Movedis = Velocity*dt; 
            
            if SteeringCar == 0
                delta_phi_trailer = phicar - phitrailer;
                obj.Xcar= [obj.Xcar, x + Movedis * cos(phicar)];
                obj.Ycar = [obj.Ycar, y + Movedis * sin(phicar)];
                obj.Phicar = [obj.Phicar, obj.Phicar(end)];
                obj.Phitrailer = [obj.Phitrailer, obj.Phitrailer(end) + Movedis/Trailerbase*sin(delta_phi_trailer)];
                
%                 obj.Xcar= x + Movedis * cos(phicar);
%                 obj.Ycar = y + Movedis * sin(phicar);
%                 obj.Phicar = obj.Phicar(end);
%                 obj.Phitrailer = obj.Phitrailer(end) + Movedis/Trailerbase*sin(delta_phi_trailer);
            else
                
                Rc = Wheelbase / tan(SteeringCar);            
                angle = Movedis/Rc;                
                delta_phi_trailer = phicar - phitrailer;                
                v_car = Movedis;                
                v1 = v_car*sin(delta_phi_trailer);                
                v2 = v_car*Hitchdis/Rc*cos(delta_phi_trailer);                
                delta_phi = (v1-v2)/Trailerbase;
                
                obj.Xcar= [obj.Xcar, x + Movedis * cos(phicar)];
                obj.Ycar = [obj.Ycar, y + Movedis * sin(phicar)];
                obj.Phicar = [obj.Phicar, obj.Phicar(end) + angle];
                obj.Phitrailer = [obj.Phitrailer, obj.Phitrailer(end)+ delta_phi];
                
%                 obj.Xcar= x + Movedis * cos(phicar);
%                 obj.Ycar = y + Movedis * sin(phicar);
%                 obj.Phicar = obj.Phicar(end) + angle;
%                 obj.Phitrailer = obj.Phitrailer(end) + delta_phi;
            end
            
            [xt, yt] = obj.calcTrailerPos();
            if ~isempty(obj.AniFig) && DrawCar
                h = obj.AniFig;
                hold on
                obj.drawCar(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Steering(end));
                obj.drawTrailer(h, obj.Xcar(end), obj.Ycar(end), obj.Phicar(end), obj.Phitrailer(end));
            end
            if ~isempty(obj.AniFig) && ~DrawCar
                h = obj.AniFig;
                hold on
                figure(h)
                plot(xt, yt, 'b*')
            end
        end
        
        function vectorNew = changePos(obj, x, y, theta, vector)
            for i = 1 : length(vector)
                vectorNew(i,:) = [x+ vector(i,1)* cos(theta)- vector(i,2)*sin(theta), y + vector(i,2)*cos(theta) + vector(i,1)*sin(theta)];
            end
        end
    end
end