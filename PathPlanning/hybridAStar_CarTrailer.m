classdef hybridAStar_CarTrailer < handle
    
    % Hybrid AStar Algorithm for Car Trailer System
    %
    % Author: Junyu Zhou
    %
    % Reference: Practical Search Techniques in Path Planning for Autonomous Driving
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
        
        % map parameters
        MapDimX
        MapDimY
        StartPoint
        GoalPoint
        Obstacle
        
        % resolution of A* algorithm
        TickXY
        TickAngle
        
        % calculated path
        Path
        SteeringWheel
        
    end
    
    methods
        
        function obj = hybridAStar_CarTrailer(CarLength, CarWidth, WheelBase, ...
                WheelTrack, TrailerBase, TrailerTrack, CarHitch)
            if nargin == 7
                obj.CarLength = CarLength;
                obj.CarWidth = CarWidth;
                obj.CarHitch = CarHitch;
                obj.WheelBase = WheelBase;
                obj.WheelTrack = WheelTrack;
                obj.TrailerBase = TrailerBase;
                obj.TrailerTrack = TrailerTrack;
            else
                obj.CarLength = 3.5;
                obj.CarWidth = 2;
                obj.CarHitch = 0.5;
                obj.WheelBase = 2.5;
                obj.WheelTrack = 1.6;
                obj.TrailerBase = 2;
                obj.TrailerTrack = 1.5;
            end
            
            obj.StartPoint = [14, 14, 0, 0];
            obj.GoalPoint = [14, 18, 0, 0];
            obj.MapDimX = 20;
            obj.MapDimY = 20;
            obj.Obstacle = [];
            obj.TickXY = 0.5;
            obj.TickAngle = 5/180*pi;
            obj.Path = [];
            obj.SteeringWheel = [];
        end
        
        function setStartPoint(obj, x, y, phi_car, phi_trailer)
            obj.StartPoint = [x y phi_car phi_trailer];
        end
        
        function setGoalPoint(obj, x, y)
            obj.GoalPoint = [x y phi_car phi_trailer];
        end
        
        function setMapDim(obj, x, y)
            obj.MapDimX = x;
            obj.MapDimY = y;
            
            boundary=[];
            for i1=0: y
                boundary=[boundary;[0 i1]];
            end
            
            for i2=0: x
                boundary=[boundary;[i2 0]];
            end
            
            for i3=0: y
                boundary=[boundary;[x i3]];
            end
            
            for i4=0: x
                boundary=[boundary;[i4 y]];
            end
            
            obj.Obstacle = [obj.Obstacle; boundary];
        end
        
        function [path, steeringwheel] = hybridAstar(obj)
            openv = [];
            costv = 0;
            closev = zeros(1,12);
            
            openv = [openv;  obj.StartPoint costv + obj.h(obj.StartPoint,obj.GoalPoint)];
            
            found = false;
            
            Numexpand = 0;
            
            
            while ~found
                
                [ costvalue, id] = min(openv(:,5));
                xv = openv(id, 1);
                yv = openv(id, 2);
                yawv = openv(id, 3);
                yawt = openv(id, 4);
                node = [xv, yv, yawv, yawt, costvalue];
                openv(id,:) = [];
                
                
                if obj.isSamePosi(obj.GoalPoint, node(1:4))
                    node
                    found = true;
                end
                
                [mgroup, steeringgroup] = obj.moveNextNode(node);
                
                for i1 = 1:length(mgroup(:,1))
                    
                    m = mgroup(i1,:);
                    steering = steeringgroup(i1,:);
                    idInt = obj.calId(m);
                    if m(1) > 0 && m(1) <= obj.MapDimX && m(2) > 0 && m(2) <= obj.MapDimY && abs(m(4)-m(3))<= 45/180*pi
                        
                        if ~obj.checkTrailer(m(1), m(2), m(4)) && ~obj.checkCar(m(1), m(2), m(3))
                            
                            id = find(closev(:,11) == idInt);
                            if ~isempty(id)
                                if (m(5) < closev(id, 5))
                                    
                                    openv = [openv; m];
                                    closev(id, 1:5) = m;
                                    closev(id, 6:10) = node;
                                    closev(id,11) = idInt;
                                    closev(id,12) = steering;
                                    
                                    Numexpand = Numexpand +1
                                end
                            else
                                openv = [openv; m];
                                closev = [closev; m, node, idInt, steering];
                                
                                Numexpand = Numexpand +1
                            end
                        end
                    end
                end
            end
            
            [path, steeringwheel] = obj.getPaths(closev, obj.StartPoint, node(1:4));
            
            obj.Path = path;
            obj.SteeringWheel = steeringwheel;
            
        end
        
        
        function animationTrailer(obj, VideoName)
            
            path = obj.Path;
            steeringwheel = obj.SteeringWheel;
            obstacle = obj.Obstacle;
            DimX = obj.MapDimX;
            DimY = obj.MapDimY;
            
            h = figure;
            set(gcf,'color','w');

            hold off;
            if length(obstacle)>=1
                plot(obstacle(:,1),obstacle(:,2),'ok');hold on;
            end
            
            plot(path(:,1),path(:,2),'-r');hold on;
            for i=1:length(path(:,1))
                obj.drawCar(h, path(i,1), path(i,2), path(i,3), steeringwheel(i))
                obj.drawTrailer(h, path(i,1), path(i,2), path(i,3), path(i,4))
            end
            axis([0-0.5 DimX+0.5 0-0.5 DimY+0.5])
            xlabel('[m]')
            ylabel('[m]')
            grid on;
            
            
            if nargin > 1
                
                writerObj = VideoWriter([VideoName '.avi']);
                
                writerObj.FrameRate = 24*2;
                % set the seconds per image
                % open the video writer
                open(writerObj);
                % write the frames to the video
                h1 = figure;
                
                set(gcf,'position',[200,100,800,750])
                set(gcf,'color','w');

                for i=1:length(path(:,1))
                    if i == 1
                        hold off;
                        if length(obstacle)>=1
                            plot(obstacle(:,1),obstacle(:,2),'ok');hold on;
                        end
                        
                        plot(path(:,1),path(:,2),'-r');hold on;
                        
                        obj.drawCar(h1, path(i,1), path(i,2), path(i,3), steeringwheel(i))
                        obj.drawTrailer(h1, path(i,1), path(i,2), path(i,3), path(i,4))
                        
                        axis([0-0.5 DimX+0.5 0-0.5 DimY+0.5])
                        xlabel('[m]')
                        ylabel('[m]')
                        grid on;
                        frame = getframe(h1);
                        writeVideo(writerObj, frame);
                    else
                        for k=1:24
                            hold off;
                            if length(obstacle)>=1
                                plot(obstacle(:,1),obstacle(:,2),'ok');hold on;
                            end
                            
                            plot(path(:,1),path(:,2),'-r');hold on;
                            
                            
                            xpath = (path(i,1) - path(i-1,1))/24*k + path(i-1,1);
                            ypath = (path(i,2) - path(i-1,2))/24*k + path(i-1,2);
                            
                            if (path(i,3) - path(i-1,3)) >= -pi && (path(i,3) - path(i-1,3)) <= pi
                                carori = (path(i,3) - path(i-1,3))/24*k + path(i-1,3);
                            else
                                if (path(i,3) - path(i-1,3)) < -pi
                                    interval =  (path(i,3) - path(i-1,3)) + 2*pi;
                                    carori = path(i,3) - interval + interval/24*k;
                                else
                                    interval =  (path(i,3) - path(i-1,3)) - 2*pi;
                                    carori = path(i,3) - interval + interval/24*k;
                                end
                            end
                            
                            if (path(i,4) - path(i-1,4)) >= -pi && (path(i,4) - path(i-1,4)) <= pi
                                trailerori = (path(i,4) - path(i-1,4))/24*k + path(i-1,4);
                            else
                                if (path(i,4) - path(i-1,4)) < -pi
                                    interval =  (path(i,4) - path(i-1,4)) + 2*pi;
                                    trailerori = path(i,4) - interval + interval/24*k;
                                else
                                    interval =  (path(i,4) - path(i-1,4)) - 2*pi;
                                    trailerori = path(i,4) - interval + interval/24*k;
                                end
                            end
                            
                            
                            obj.drawCar(h1, xpath, ypath, carori, steeringwheel(i))
                            obj.drawTrailer(h1, xpath, ypath, carori, trailerori)
                            
                            axis([0-0.5 DimX+0.5 0-0.5 DimY+0.5])
                            xlabel('[m]')
                            ylabel('[m]')
                            grid on;
                            frame = getframe(h1);
                            writeVideo(writerObj, frame);
                        end
                    end
                end
                
                % close the writer object
                close(writerObj);
            end
        end
        
        function setObstacle(obj, obstacles)
            if nargin == 1
                ob = [];
                for i1 = 0:8
                    ob = [ob; [i1 17]];
                end
                
                for i1 = 18:20
                    ob = [ob; [i1 17]];
                end
                
                for i1 = 18:20
                    ob = [ob; [8 i1]];
                end
                
                for i1 = 18:20
                    ob = [ob; [18 i1]];
                end
                
                
                for i1 = 0:8
                    ob = [ob; [i1 7]];
                end
                
                for i1 = 12:20
                    ob = [ob; [i1 7]];
                end
                
                for i1 = 0:7
                    ob = [ob; [8 i1]];
                end
                
                for i1 = 0:7
                    ob = [ob; [12 i1]];
                end
                
                for i1=0:(obj.MapDimX)
                    ob=[ob;[i1 12]];
                end
                
                obj.Obstacle = [obj.Obstacle; ob];
            else
                obj.Obstacle = [obj.Obstacle; obstacles];
            end
            
        end
        
    end
    
    methods (Access = private)
        
        function result = h(obj, a, b) % heuristic function of 2D Euclid distance
            result=norm(a(1:2)-b(1:2));
        end
        
        
        function result = isSamePosi(obj, a, b)
            result=false;
            d=a(1:4)-b;
            if abs(d(1))<obj.TickXY/2 && abs(d(2))<obj.TickXY/2 && abs(d(3))<obj.TickAngle && abs(d(4)) < obj.TickAngle
                result=true;
            end
        end
        
        function idInt = calId(obj, m)
            [x, y, yawv, yawt] = obj.roundPos(m);
            idInt = x*100 +y + yawv * 10e8 + yawt * 10e6;
        end
        
        function [x, y, yawv, yawt] = roundPos(obj, node)
            Tick = obj.TickXY;
            AngleTick = obj.TickAngle;
            
            x = round(node(1)/Tick)*Tick;
            y = round(node(2)/Tick)*Tick;
            yawv = floor(node(3)/AngleTick)*AngleTick;
            yawt = floor(node(4)/AngleTick)*AngleTick;
        end
        
        function angle= roundAngle(obj, angle)
            while angle>pi
                angle=angle-2*pi;
            end
            while angle<=-pi
                angle=angle+2*pi;
            end
        end
        
        function [path, steeringwheel] = getPaths(obj, close, start, goal)
            id = obj.findPoint(close, goal);
            path = [goal];
            steeringwheel = [0];
            path = [close(id, 6:9); path];
            steeringwheel = [close(id,12); steeringwheel];
            prepoint = close(id, 6:9);
            %found = false;
            while ~obj.isSamePosi(start,prepoint)
                for io=1:length(close(:,1))
                    
                    [x1, y1, orientation1, ot1] = obj.roundPos(close(io,1:4));
                    [x2, y2, orientation2, ot2] = obj.roundPos(prepoint);
                    
                    if obj.isSamePosi([x1, y1, orientation1, ot1],[x2, y2, orientation2, ot2])
                        id=io;
                        prepoint = close(id, 6:9);
                        break;
                    end
                end
                path = [close(id, 6:9); path];
                steeringwheel = [close(id,12); steeringwheel];
            end
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
        
        function vectorNew = changePos(obj, x, y, theta, vector)
            for i = 1 : length(vector)
                vectorNew(i,:) = [x+ vector(i,1)* cos(theta)- vector(i,2)*sin(theta), y + vector(i,2)*cos(theta) + vector(i,1)*sin(theta)];
            end
        end
        
        function id = findPoint(obj, close, point)
            id = [];
            idx = find(close(:,1) == point(1));
            for k1 = 1:length(idx)
                if close(idx(k1),2) == point(2) && close(idx(k1), 3) == point(3) && close(idx(k1), 4) == point(4)
                    id = idx(k1);
                    return
                end
            end
            
        end
        
        function status = checkCar_Test(obj, x, y, theta)
            
            carwidth = obj.CarWidth;
            carlength = obj.CarLength;
            obstacle = obj.Obstacle;
            
            rear = linspace(-carwidth/2, carwidth/2, 50);
            rear = [-0.3*ones(length(rear),1), rear'];
            
            front = linspace(-carwidth/2, carwidth/2, 50);
            front = [carlength*ones(length(front),1), front'];
            
            left = linspace(-0.3, carlength, 50);
            left = [ left', carwidth/2*ones(length(left),1),];
            
            right = linspace(-0.3, carlength, 50);
            right = [ right', -carwidth/2*ones(length(right),1),];
            
            left = obj.changePos(x, y, theta, left);
            right = obj.changePos(x, y, theta, right);
            front = obj.changePos(x, y, theta, front);
            rear = obj.changePos(x, y, theta, rear);
            
            
            list = [left; right; front; rear];
            
            status = false;
            for j1 = 1 : length(list)
                for j2 = 1 : length(obstacle)
                    if norm(list(j1, :) - obstacle(j2,:)) < 0.2
                        status = true;
                        return;
                    end
                end
                
            end
        end
        
        function status = checkCar(obj, x, y, theta)
            
            obstacle = obj.Obstacle;
            
            carwidth = obj.CarWidth;
            carlength = obj.CarLength;
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
            
            list = [left; right; front; rear];
            
            status = false;
            for j1 = 1 : length(list)
                if list(j1,1) >= 20-0.2
                    status = true;
                    return;
                end
                
                if list(j1,1) <= 0+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) >= 20-0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 0+0.2
                    status = true;
                    return;
                end
                if list(j1,2) <= 12+0.2
                    status = true;
                    return;
                end
                
                %                 if list(j1,2) >=  12-0.2
                %                     status = true;
                %                     return;
                %                 end
                
                if list(j1,2) >= 17 && list(j1, 1) <= 8+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) >= 17 && list(j1, 1) >= 18-0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 7 && list(j1, 1) <= 8+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 7 && list(j1, 1) >= 12-0.2
                    status = true;
                    return;
                end
                
            end
        end
        
        function status = checkTrailer_Test(obj, x, y, theta_car, theta_trailer)
            
            obstacle = obj.Obstacle;
            
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
            
            
            list = [rod; rear; wl; wr];
            
            status = false;
            for j1 = 1 : length(list)
                for j2 = 1 : length(obstacle)
                    if norm(list(j1, :) - obstacle(j2,:)) < 0.2
                        status = true;
                        return;
                    end
                end
            end
        end
        
        
        function status = checkTrailer(obj, x, y, theta)
            
            trailerwidth = obj.TrailerTrack;
            trailerlength = obj.TrailerBase;
            obstacle = obj.Obstacle;
            
            rear = linspace(-trailerwidth/2, trailerwidth/2, 50);
            rear = [-trailerlength*ones(length(rear),1), rear'];
            
            rod = linspace(0, -trailerlength, 50);
            rod = [ rod', 0*ones(length(rod),1),];
            
            rod = obj.changePos(x, y, theta, rod);
            rear = obj.changePos(x, y, theta, rear);
            
            
            list = [rod; rear];
            
            status = false;
            for j1 = 1 : length(list)
                if list(j1,1) >= 20-0.2
                    status = true;
                    return;
                end
                
                if list(j1,1) <= 0+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) >= 20-0.2
                    status = true;
                    return;
                end
                
                
                if list(j1,2) <= 0+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 12+0.5
                    status = true;
                    return;
                end
                
                
                %                 if list(j1,2) >= 12 - 0.2
                %                     status = true;
                %                     return;
                %                 end
                
                
                if list(j1,2) >= 17 && list(j1, 1) <= 8+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) >= 17 && list(j1, 1) >= 18-0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 7 && list(j1, 1) <= 8+0.2
                    status = true;
                    return;
                end
                
                if list(j1,2) <= 7 && list(j1, 1) >= 12-0.2
                    status = true;
                    return;
                end
            end
        end
        
        function [m, steering]=moveNextNode(obj, node)
            
            
            Tick = obj.TickXY;
            Wheelbase = obj.WheelBase;
            Trailerbase = obj.TrailerBase;
            Hitchdis = obj.CarHitch;
            
            movedis=[Tick*sqrt(2)+0.0001,-Tick*sqrt(2)-0.0001];
            steering=-30:10:30;
            
            rad=(steering)/180*pi;
            
            step = 10;
            
            m = [];
            
            steering = [];
            
            nodeStart = node;
            
            for i=1:length(rad)
                
                for j = 1:length(movedis)
                    
                    node = nodeStart;
                    
                    if rad(i) == 0
                        angle = 0;
                        costfact = 1;
                    else
                        Rc = Wheelbase/tan(rad(i));
                        angle = movedis(j)/step/Rc;
                        costfact = 2;
                    end
                    
                    if movedis(j)> 0
                        costfact = costfact + 0;
                    else
                        costfact = costfact + 5;
                    end
                    
                    for k = 1:step
                        
                        if rad(i) == 0
                            phi_car=node(3);
                            
                            delta_phi_trailer = (node(3)-node(4));
                            
                            delta_phi = movedis(j)/step/Trailerbase*sin(delta_phi_trailer);
                            
                            next=[movedis(j)/step*cos(phi_car) movedis(j)/step*sin(phi_car) angle delta_phi costfact/step];
                        else
                            Rc = Wheelbase / tan(rad(i));
                            
                            phi_car=node(3);
                            
                            delta_phi_trailer = (node(3)-node(4));
                            
                            v_car = movedis(j)/step;
                            
                            v1 = v_car*sin(delta_phi_trailer);
                            
                            v2 = v_car*Hitchdis/Rc*cos(delta_phi_trailer);
                            
                            delta_phi = (v1-v2)/Trailerbase;
                            
                            next=[movedis(j)/step*cos(phi_car) movedis(j)/step*sin(phi_car) angle delta_phi costfact/step];
                            
                        end
                        node = node + next;
                        
                        node(3)=obj.roundAngle(node(3));
                        node(4)=obj.roundAngle(node(4));
                        
                    end
                    
                    node(5) = node(5) + obj.h(node(1:3),obj.GoalPoint)-obj.h(nodeStart(1:3),obj.GoalPoint);
                    
                    m = [m; node];
                    
                    steering = [steering;rad(i)];
                    
                end
            end
            
        end
    end
end