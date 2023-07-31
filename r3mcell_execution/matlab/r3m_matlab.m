% r3m_matlab.m

% PREVIOUS STEP! Execute r3m_init.m script.

% ============================================================ %
% =========================== MAIN =========================== %
% ============================================================ %

global id
global ExecTime
global Error
global Product
global FDBMessage
global FDBSuccess

% 1. Publish to /r3m_MATLAB -> Execute RECIPE:
RECIPE = ros2message(r3m_PUBLISHER);
RECIPE.data = '5'; 
send(r3m_PUBLISHER,RECIPE)

% 2. Subscribe to FEEDBACK:
[scanData,status,statustext] = receive(r3m_SUBSCRIBER,15);

% PRINT:
PRINT = 'R3M - RECIPE EXECUTION from Matlab:';
disp(PRINT)
PRINT = ['Recipe NUMBER -> ', num2str(id)];
disp(PRINT)
PRINT = ['Feedback MSG -> ', FDBMessage.data];
disp(PRINT)
PRINT = ['ExecutionTime -> ', num2str(ExecTime)];
disp(PRINT)
PRINT = ['Error -> ', num2str(Error)];
disp(PRINT)
for i = 1:length(Product)
    PRINT = ['Product NAME -> ', Product(i).name.data];
    disp(PRINT)
    PRINT = ['   - CurrentPose: (x: ', num2str(Product(i).currentpose.x), ', y: ', num2str(Product(i).currentpose.y), ', z: ', num2str(Product(i).currentpose.z), ', qx: ', num2str(Product(i).currentpose.qx), ', qy: ', num2str(Product(i).currentpose.qy), ', qz: ', num2str(Product(i).currentpose.qz), ', qw: ', num2str(Product(i).currentpose.qw), ')'];
    disp(PRINT)
    PRINT = ['   - PreviousPose: (x: ', num2str(Product(i).previouspose.x), ', y: ', num2str(Product(i).previouspose.y), ', z: ', num2str(Product(i).previouspose.z), ', qx: ', num2str(Product(i).previouspose.qx), ', qy: ', num2str(Product(i).previouspose.qy), ', qz: ', num2str(Product(i).previouspose.qz), ', qw: ', num2str(Product(i).previouspose.qw), ')'];
    disp(PRINT)
end
