%% Initial conditions 

m_p=0.100 % [kg], mass of pendulum
l_r=0.100 % [m], length of rotary arm 
l_p=0.100 % [m], length of pendulum link

j_p=4.908738521234051*(10^6) % [m], moment of inertia about the link's center of mass
j_r=4.908738521234051*(10^6) % [m], moment of inertia of arm

b_r=0.001 % friction coefficient of motor pivot
b_p=0.0001 % friction coefficient of pendulum pivot 
tau=1 % [Nm], torque
g=9.8 % [m/s^2]

%First Linear Equation of Motion WRT forces acting on rotary arm. 
ans=tau*b_r
ans2=(m_p*(l_r^2)+j_r)-1/2*(m_p*l_p*l_r)

%Second Linear Equation of Motion WRT forces acting on the pendulum. 
ans3=-b_p
ans4=-1/2*(m_p*l_p*l_r)+(j_p+1/4*(m_p)*l_p^2)-1/2*(m_p*l_p*g)

%Static Gain
K= j_p*m_p*(l_r^2)+(j_r*j_p)+1/4*(j_r*m_p*(l_p^2))

%Acceleration terms for equations of motion
theta_2=1/K*(-j_p+1/4*(m_p*l_p^2))*b_r-1/2*(m_p*l_p*l_r*b_r)+1/4*((m_p^2)*(l_p^2)*l_r*g)+tau*(j_p+1/4*(m_p*(l_p^2)))
alpha_2=1/K*(1/2*(m_p*l_p*l_r*b_r))-(j_r+m_p*(l_r^2))*b_p+1/2*(m_p*l_p*g*(j_r+m_p*(l_r^2)))+1/2*(m_p*l_p*l_r*tau)

%%

m1=0.2;% [kg]
m2=0.7; % [kg]
L1=1.4;% [m]
L2=0.5;% [m]

%create base input to excite the system, must be the same as in Simulink

u_base=@(x,t) 5*sin(2*pi*t)+10; % [N*m]
% x is state vector [angle(base, pen), vel(base, pen)];

Mod2pi=@(a) atan2(sin(a),cos(a));

if ~isempty(whos('simout'))

% output of Simulink is simout
t_sim=simout(:,1);
Theta1_sim   =simout(:,2); %base
Theta1d_sim  =simout(:,3);
Theta1dd_sim =simout(:,4);
Theta2_sim   =simout(:,5); %pendulum
Theta2d_sim  =simout(:,6);
Theta2dd_sim =simout(:,7);
U_sim        =simout(:,8);
Flag_Pen.SimResults=true;
x_o=simout(1,[2,5,3,6]);
else
    fprintf('Run RotPen_Test_Submitt.mdl to get SimMechanics results for comparison\n');
    Flag_Pen.SimResults=false;
    t_sim=linspace(0,1,100);
    x_o=[0,0,0,1].';
end

%Find Response 
MassMatrixInv=@(a2) [(m1*L1^2/3+m2*L1^2+4/3*m2*L2^2*sin(a2)^2)*(-m2*L1*L2*cos(a2)); 
    (-m2*L1*L2*cos(a2)), 4/3*m2*L2^2]^-1;
Dynamics=@(t,x) [x(3);x(4);(...
    MassMatrixInv(x(2))*([u_base(x,t)+...
    -x(3)*x(4)*4/3*m2*L2^2*(2*sin(x(2))*cos(x(2)))-x(4)^2*m2*L1*L2*sin(x(2));...
    +x(3)^2*4/3*m2*L2^2*sin(x(2))*cos(x(2))+m2*g*L2*sin(x(2))]))];

[t,Theta]=ode45(Dynamics,t_sim,x_o);

% Recalculate derivatives and control for comparison
Theta_dot=zeros(size(Theta));
U=zeros(size(t));
for angie=1:length(t)
    Theta_dot(angie,:)=Dynamics(t(angie),Theta(angie,:)).';
    U(angie)=u_base(Theta(angie,:),t(angie));
end