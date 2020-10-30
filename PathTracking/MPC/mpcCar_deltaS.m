V = 20;
x0 = [0; -1; 0; V; 0]; 
u0 = [0; 0];

%%
% Discretize the continuous-time model using the zero-order holder method
% in the |obstacleVehicleModelDT| function.
Ts = 0.02;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = VehicleModelDT_deltaS(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Throttle','D_Delta'};
dsys.StateName = {'X','Y','Theta','V', 'Delta'};
dsys.OutputName = dsys.StateName;

%% MPC Design at the Nominal Operating Point
% Design a model predictive controller that can make the ego car maintain
% a desired velocity and stay in the middle of the center lane.
status = mpcverbosity('off');
mpcobj = mpc(dsys);

%% 
% The prediction horizon is |25| steps, which is equivalent to 0.5 seconds.
mpcobj.PredictionHorizon = 60;%25;
mpcobj.ControlHorizon = 2;%5;

%%
% To prevent the ego car from accelerating or decelerating too quickly, add
% a hard constraint of 0.2 (m/s^2) on the throttle rate of change.
mpcobj.ManipulatedVariables(1).RateMin = -0.2*Ts; 
mpcobj.ManipulatedVariables(1).RateMax = 0.2*Ts;

%%
% Similarly, add a hard constraint of 6 degrees per second on the steering
% angle rate of change.
mpcobj.ManipulatedVariables(2).RateMin = -pi/30*Ts;
mpcobj.ManipulatedVariables(2).RateMax = pi/30*Ts;

%%
% Scale the throttle and steering angle by their respective operating
% ranges.
mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;

%%
% Since there are only two manipulated variables, to achieve zero
% steady-state offset, you can choose only two outputs for perfect
% tracking. In this example, choose the Y position and velocity by setting
% the weights of the other two outputs (X and theta) to zero. Doing so lets
% the values of these other outputs float.
mpcobj.Weights.OutputVariables = [0.2 30 0.2 1 0];

%% 
% Update the controller with the nominal operating condition. For a
% discrete-time plant:
%
% * |U = u0|
% * |X = x0|
% * |Y = Cd*x0 + Dd*u0|
% * |DX = Ad*X0 + Bd*u0 - x0|
%
mpcobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);

% %% Specify Mixed I/O Constraints for Obstacle Avoidance Maneuver
% % There are different strategies to make the ego car avoid an obstacle on
% % the road. For example, a real-time path planner can compute a new path
% % after an obstacle is detected and the controller follows this path.
% %
% % In this example, use a different approach that takes advantage of the
% % ability of MPC to handle constraints explicitly. When an obstacle is
% % detected, it defines an area on the road (in terms of constraints) that
% % the ego car must not enter during the prediction horizon. At the next
% % control interval, the area is redefined based on the new positions of
% % the ego car and obstacle until passing is completed.
% %
% % To define the area to avoid, use the following mixed input/output
% % constraints:
% %
% %   E*u + F*y <= G
% % 
% % where |u| is the manipulated variable vector and |y| is the output
% % variable vector. You can update the constraint matrices |E|, |F|, and |G|
% % when the controller is running.
% 
% %%
% % The first constraint is an upper bound on $y$ ($y \le 6$ on this
% % three-lane road).
% E1 = [0 0];
% F1 = [0 1 0 0]; 
% G1 = laneWidth*lanes/2;
% 
% %%
% % The second constraint is a lower bound on $y$ ($y \ge -6$ on this
% % three-lane road).
% E2 = [0 0];
% F2 = [0 -1 0 0]; 
% G2 = laneWidth*lanes/2;
% 
% %%
% % The third constraint is for obstacle avoidance. Even though no obstacle
% % is detected at the nominal operating condition, you must add a "fake"
% % constraint here because you cannot change the dimensions of the
% % constraint matrices at run time. For the fake constraint, use a
% % constraint with the same form as the second constraint.
% E3 = [0 0];
% F3 = [0 -1 0 0]; 
% G3 = laneWidth*lanes/2;
% 
% %%
% % Specify the mixed input/output constraints in the controller using the
% % |setconstraint| function.
% setconstraint(mpcobj,[E1;E2;E3],[F1;F2;F3],[G1;G2;G3],[1;1;0.1]);

%% Simulate Controller
% In this example, you use an adaptive MPC controller because it handles
% the nonlinear vehicle dynamics more effectively than a traditional MPC
% controller. A traditional MPC controller uses a constant plant model.
% However, adaptive MPC allows you to provide a new plant model at each
% control interval. Because the new model describes the plant dynamics more
% accurately at the new operating condition, an adaptive MPC controller
% performs better than a traditional MPC controller.
%
% Also, to enable the controller to avoid the safe zone surrounding the
% obstacle, you update the third mixed constraint at each control interval.
% Basically, the ego car must be above the line formed from the ego car to
% the upper left corner of the safe zone. For more details, open
% |obstacleComputeCustomConstraint|.

%%
% Use a constant reference signal.
refSignal = [0 0 0 V 0];

%%
% Initialize plant and controller states.
x = x0;
u = u0;
egoStates = mpcstate(mpcobj);

%%
% The simulation time is |4| seconds.
T = 0:Ts:10;

%%
% Log simulation data for plotting.
saveSlope = zeros(length(T),1);
saveIntercept = zeros(length(T),1);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));

%%
% Run the simulation.
for k = 1:length(T)
    % Obtain new plant model and output measurements for interval |k|.
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = VehicleModelDT_deltaS(Ts,x,u);
    measurements = Cd * x + Dd * u;
    ympc(k,:) = measurements';
    
%     % Determine whether the vehicle sees the obstacle, and update the mixed
%     % I/O constraints when obstacle is detected.
%     detection = obstacleDetect(x,obstacle,laneWidth);
%     [E,F,G,saveSlope(k),saveIntercept(k)] = ...
%         obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes); 
   
    % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
    
    [refSignal, PathRef_x, PathRef_y] = refPath(x, Ts, V, (k-1)*Ts);    
    refSignal = [refSignal zeros(size(refSignal,1),1)];
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
%     options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
    % Compute optimal moves using the updated plant, nominal conditions,
    % and constraints.
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,refSignal);
    umpc(k,:) = u';
    
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

mpcverbosity(status);

%% Analyze Results
figure;
subplot(2,1,1)
hold on
plot(PathRef_x, PathRef_y, 'r');
plot(ympc(:,1),ympc(:,2),'-k');
xlabel('x [m]');
ylabel('y [m]');
grid on
xlim([min(ympc(:,1)) max(ympc(:,1))])

subplot(2,1,2)
plot(T, ympc(:,5)*180/pi)
grid on
xlabel('time [s]')
ylabel('Steering angle [°]')