%Initialize API

sim=remApi('remoteApi');

% using the prototype file (remoteApiProto.m)

sim.simxFinish(-1);

% just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)       

disp('Connected to remote API server');       

sim.simxGetStringSignal(clientID,'distance',sim.simx_opmode_streaming);               

set_param('ballclmod', 'SimulationCommand', 'start')               

    while(1)  % In this while loop, we will have the communication

        [errorCode,r_mat]=sim.simxGetStringSignal(clientID,'distance',sim.simx_opmode_buffer);           

        %%if errorCode is not vrep.simx_return_ok, this does not mean there is an error:            

       %%it could be that the first streamed values have not yet arrived, or that the signal            

       %%is empty/non-existent           

            set_param('ballclmod/Constant','Value',num2str(r_mat));  %ballclmod is the model file and Constant is the block's name, r_mat is the variable to send.      

            pause(.01);

            theta = get_param('ballclmod/To Workspace','RuntimeObject'); % We receive the sensor data from Simulink model ballclmod and To Workspace block via RuntimeObject

            theta.InputPort(1).Data;                                                                      % Receive the data

   end