if ~mpcchecktoolboxinstalled('optim')
    disp('Optimization Toolbox is required to run this example.')
    return
end


%% Create Nonlinear MPC Controller
nx = 4;
ny = 2;
nu = 1;
nlobj = nlmpc(nx, ny, nu);

%%
Ts = 0.02;
nlobj.Ts = Ts;

%%
nlobj.PredictionHorizon = 60;

%%
nlobj.ControlHorizon = 5;

%% Specify Nonlinear Plant Model
% The major benefit of nonlinear model predictive control is that it uses a
% nonlinear dynamic model to predict plant behavior in the future across a
% wide range of operating conditions.
nlobj.Model.StateFcn = "VehicleTrailerAModelDT0";

%%
% To use a discrete-time model, set the |Model.IsContinuousTime| property
% of the controller to |false|.
nlobj.Model.IsContinuousTime = false;

%%

nlobj.Model.NumberOfParameters = 1;

%%

nlobj.Model.OutputFcn = 'VehicleTrailerModelOutputFcn';

%%
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [0 1 0 0;0 0 0 1];


%% Define Cost and Constraints
nlobj.Weights.OutputVariables = [3 3];
nlobj.Weights.ManipulatedVariablesRate = 0.01;

%%
nlobj.OV(2).Min = -45/180*pi;
nlobj.OV(2).Max = 45/180*pi;

%%
nlobj.MV.Min = -30/180*pi;
nlobj.MV.Max = 30/180*pi;

%% Validate Nonlinear MPC Controller
x0 = [0 ; 0.3; 0; 15/180*pi];
u0 = 0;
validateFcns(nlobj,x0,u0,[],{Ts});


x = [0;0.3;0;3/180*pi];
y = [x(1);x(2)];

%%
% |mv| is the optimal control move computed at any control interval.
mv = 0;

%%
% In the first stage of the simulation, the pendulum swings up from a
% downward equilibrium position to an inverted equilibrium position. The
% state references for this stage are all zero.
yref = [0 0];

%%
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

%%
% Run the simulation for |20| seconds.
Duration = 5;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
uHistory = [0];
refSignal = yref;
xk = x;
for ct = 1:(Duration/Ts)
    % Set references
    % Correct previous prediction using current measurement.
    xk = x;
    
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,refSignal,[],nloptions);
    x = VehicleTrailerAModelDT0(x,mv,Ts);
    %y = VehicleModelOutputFcn(x, mv, Ts);
    xHistory = [xHistory x]; 
    uHistory = [uHistory mv];
    waitbar(ct*Ts/Duration,hbar);
end
close(hbar)

%%
% Plot the closed-loop response.
figure
subplot(3,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time [s]')
ylabel('x [m]')
title('cart position x')
grid on
subplot(3,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time [s]')
ylabel('y [m]')
title('cart positon y')
grid on
subplot(3,2,3)
plot(0:Ts:Duration,xHistory(4,:)*180/pi)
hold on
plot(0:Ts:Duration, (0:Ts:Duration)*0)
xlabel('time [s]')
ylabel('theta trailer [°]')
title('trailer head angle')
legend('current', 'desired')
grid on
subplot(3,2,4)
plot(xHistory(1,:), xHistory(2,:))
hold on 
plot(xHistory(1,:),xHistory(1,:)*0,'r')
xlabel('x [m]')
ylabel('y [m]')
title('XY postion')
legend('current', 'desired')
grid on
subplot(3,2,5)
plot(0:Ts:Duration,xHistory(3,:)*180/pi)
xlabel('time [s]')
ylabel('theta [°]')
title('vehicle head angle')
grid on
subplot(3,2,6)
plot(0:Ts:Duration,uHistory(:)*180/pi)
xlabel('time [s]')
ylabel('delta [°]')
title('steering angle')
grid on