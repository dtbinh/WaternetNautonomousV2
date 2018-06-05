classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    properties (Constant)
        nautonomous_authentication_msgs_Authentication = 'nautonomous_authentication_msgs/Authentication'
        nautonomous_authentication_msgs_AuthenticationRequest = 'nautonomous_authentication_msgs/AuthenticationRequest'
        nautonomous_authentication_msgs_AuthenticationResponse = 'nautonomous_authentication_msgs/AuthenticationResponse'
        nautonomous_authentication_msgs_Encryption = 'nautonomous_authentication_msgs/Encryption'
        nautonomous_authentication_msgs_EncryptionRequest = 'nautonomous_authentication_msgs/EncryptionRequest'
        nautonomous_authentication_msgs_EncryptionResponse = 'nautonomous_authentication_msgs/EncryptionResponse'
        nautonomous_logging_msgs_LogGPS = 'nautonomous_logging_msgs/LogGPS'
        nautonomous_logging_msgs_LogIMU = 'nautonomous_logging_msgs/LogIMU'
        nautonomous_logging_msgs_LogPropulsionStatus = 'nautonomous_logging_msgs/LogPropulsionStatus'
        nautonomous_map_msgs_Crop = 'nautonomous_map_msgs/Crop'
        nautonomous_map_msgs_CropRequest = 'nautonomous_map_msgs/CropRequest'
        nautonomous_map_msgs_CropResponse = 'nautonomous_map_msgs/CropResponse'
        nautonomous_map_msgs_Load = 'nautonomous_map_msgs/Load'
        nautonomous_map_msgs_LoadRequest = 'nautonomous_map_msgs/LoadRequest'
        nautonomous_map_msgs_LoadResponse = 'nautonomous_map_msgs/LoadResponse'
        nautonomous_mission_msgs_MissionPlanAction = 'nautonomous_mission_msgs/MissionPlanAction'
        nautonomous_mission_msgs_MissionPlanActionFeedback = 'nautonomous_mission_msgs/MissionPlanActionFeedback'
        nautonomous_mission_msgs_MissionPlanActionGoal = 'nautonomous_mission_msgs/MissionPlanActionGoal'
        nautonomous_mission_msgs_MissionPlanActionResult = 'nautonomous_mission_msgs/MissionPlanActionResult'
        nautonomous_mission_msgs_MissionPlanFeedback = 'nautonomous_mission_msgs/MissionPlanFeedback'
        nautonomous_mission_msgs_MissionPlanGoal = 'nautonomous_mission_msgs/MissionPlanGoal'
        nautonomous_mission_msgs_MissionPlanResult = 'nautonomous_mission_msgs/MissionPlanResult'
        nautonomous_mission_msgs_MissionStatus = 'nautonomous_mission_msgs/MissionStatus'
        nautonomous_mission_msgs_OperationPlan = 'nautonomous_mission_msgs/OperationPlan'
        nautonomous_mpc_msgs_Obstacle = 'nautonomous_mpc_msgs/Obstacle'
        nautonomous_mpc_msgs_Obstacles = 'nautonomous_mpc_msgs/Obstacles'
        nautonomous_pose_msgs_PointWithCovarianceStamped = 'nautonomous_pose_msgs/PointWithCovarianceStamped'
        nautonomous_routing_msgs_Route = 'nautonomous_routing_msgs/Route'
        nautonomous_routing_msgs_RouteRequest = 'nautonomous_routing_msgs/RouteRequest'
        nautonomous_routing_msgs_RouteResponse = 'nautonomous_routing_msgs/RouteResponse'
        nautonomous_state_msgs_Boat = 'nautonomous_state_msgs/Boat'
        nautonomous_state_msgs_BoatCommand = 'nautonomous_state_msgs/BoatCommand'
        nautonomous_state_msgs_BoatParam = 'nautonomous_state_msgs/BoatParam'
        nautonomous_state_msgs_BoatState = 'nautonomous_state_msgs/BoatState'
        nautonomous_state_msgs_Border = 'nautonomous_state_msgs/Border'
        nautonomous_state_msgs_Borders = 'nautonomous_state_msgs/Borders'
        nautonomous_state_msgs_State = 'nautonomous_state_msgs/State'
        nautonomous_webserver_msgs_NautonomousAuthentication = 'nautonomous_webserver_msgs/NautonomousAuthentication'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(33, 1);
                msgList{1} = 'nautonomous_authentication_msgs/AuthenticationRequest';
                msgList{2} = 'nautonomous_authentication_msgs/AuthenticationResponse';
                msgList{3} = 'nautonomous_authentication_msgs/EncryptionRequest';
                msgList{4} = 'nautonomous_authentication_msgs/EncryptionResponse';
                msgList{5} = 'nautonomous_logging_msgs/LogGPS';
                msgList{6} = 'nautonomous_logging_msgs/LogIMU';
                msgList{7} = 'nautonomous_logging_msgs/LogPropulsionStatus';
                msgList{8} = 'nautonomous_map_msgs/CropRequest';
                msgList{9} = 'nautonomous_map_msgs/CropResponse';
                msgList{10} = 'nautonomous_map_msgs/LoadRequest';
                msgList{11} = 'nautonomous_map_msgs/LoadResponse';
                msgList{12} = 'nautonomous_mission_msgs/MissionPlanAction';
                msgList{13} = 'nautonomous_mission_msgs/MissionPlanActionFeedback';
                msgList{14} = 'nautonomous_mission_msgs/MissionPlanActionGoal';
                msgList{15} = 'nautonomous_mission_msgs/MissionPlanActionResult';
                msgList{16} = 'nautonomous_mission_msgs/MissionPlanFeedback';
                msgList{17} = 'nautonomous_mission_msgs/MissionPlanGoal';
                msgList{18} = 'nautonomous_mission_msgs/MissionPlanResult';
                msgList{19} = 'nautonomous_mission_msgs/MissionStatus';
                msgList{20} = 'nautonomous_mission_msgs/OperationPlan';
                msgList{21} = 'nautonomous_mpc_msgs/Obstacle';
                msgList{22} = 'nautonomous_mpc_msgs/Obstacles';
                msgList{23} = 'nautonomous_pose_msgs/PointWithCovarianceStamped';
                msgList{24} = 'nautonomous_routing_msgs/RouteRequest';
                msgList{25} = 'nautonomous_routing_msgs/RouteResponse';
                msgList{26} = 'nautonomous_state_msgs/Boat';
                msgList{27} = 'nautonomous_state_msgs/BoatCommand';
                msgList{28} = 'nautonomous_state_msgs/BoatParam';
                msgList{29} = 'nautonomous_state_msgs/BoatState';
                msgList{30} = 'nautonomous_state_msgs/Border';
                msgList{31} = 'nautonomous_state_msgs/Borders';
                msgList{32} = 'nautonomous_state_msgs/State';
                msgList{33} = 'nautonomous_webserver_msgs/NautonomousAuthentication';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(5, 1);
                svcList{1} = 'nautonomous_authentication_msgs/Authentication';
                svcList{2} = 'nautonomous_authentication_msgs/Encryption';
                svcList{3} = 'nautonomous_map_msgs/Crop';
                svcList{4} = 'nautonomous_map_msgs/Load';
                svcList{5} = 'nautonomous_routing_msgs/Route';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(1, 1);
                actList{1} = 'nautonomous_mission_msgs/MissionPlan';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
