% r3m_matlab.m
% Matlab script needed to initialise the connection w/ ROS2.

% Initialise ROS2 Publisher:
NODE = ros2node("r3m_MATLAB_NODE");
r3m_PUBLISHER = ros2publisher(NODE,"/r3m_MATLAB","std_msgs/String", reliability = "reliable");
RECIPE = ros2message(r3m_PUBLISHER);
RECIPE.data = '0';
send(r3m_PUBLISHER,RECIPE)

% Initialise ROS2 Subscriber:
r3m_SUBSCRIBER = ros2subscriber(NODE, "r3m_FEEDBACK", "r3mcell_data/Skillresult", @FeedbackCallback);

% CALLBACK FUNCTION -> FEEDBACK: 
function FeedbackCallback(MSG)
    
    global id
    global ExecTime
    global Error
    global Product
    global FDBMessage
    global FDBSuccess

    id = MSG.id;
    ExecTime = MSG.exectime;

    Error = 0.0;
    for i = 1:length(MSG.error)
        Error = Error + MSG.error(i);
    end
    Error = Error/length(MSG.error);

    for i = 1:length(MSG.product)
        
        Product(i).name = MSG.product(i).name;
        
        Pose.x = MSG.product(i).currentpose.x;
        Pose.y = MSG.product(i).currentpose.y;
        Pose.z = MSG.product(i).currentpose.z;
        Pose.qx = MSG.product(i).currentpose.qx;
        Pose.qy = MSG.product(i).currentpose.qy;
        Pose.qz = MSG.product(i).currentpose.qz;
        Pose.qw = MSG.product(i).currentpose.qw;
        Product(i).currentpose = Pose;

        Pose.x = MSG.product(i).previouspose.x;
        Pose.y = MSG.product(i).previouspose.y;
        Pose.z = MSG.product(i).previouspose.z;
        Pose.qx = MSG.product(i).previouspose.qx;
        Pose.qy = MSG.product(i).previouspose.qy;
        Pose.qz = MSG.product(i).previouspose.qz;
        Pose.qw = MSG.product(i).previouspose.qw;
        Product(i).previouspose = Pose;

    end     

    FDBMessage = MSG.message;
    FDBSuccess = MSG.success;
   
end