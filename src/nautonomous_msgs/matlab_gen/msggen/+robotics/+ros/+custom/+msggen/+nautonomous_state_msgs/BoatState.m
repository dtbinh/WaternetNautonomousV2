classdef BoatState < robotics.ros.Message
    %BoatState MATLAB implementation of nautonomous_state_msgs/BoatState
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'nautonomous_state_msgs/BoatState' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '6b850078db04fd1f7844fcc29c113e38' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPoseClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Pose') % Dispatch to MATLAB class for message type geometry_msgs/Pose
        GeometryMsgsTwistClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Twist') % Dispatch to MATLAB class for message type geometry_msgs/Twist
    end
    
    properties (Dependent)
        Pose
        Twist
        Progress
    end
    
    properties (Access = protected)
        Cache = struct('Pose', [], 'Twist', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Pose', 'Progress', 'Twist'} % List of non-constant message properties
        ROSPropertyList = {'pose', 'progress', 'twist'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = BoatState(msg)
            %BoatState Construct the message object BoatState
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function pose = get.Pose(obj)
            %get.Pose Get the value for property Pose
            if isempty(obj.Cache.Pose)
                obj.Cache.Pose = feval(obj.GeometryMsgsPoseClass, obj.JavaMessage.getPose);
            end
            pose = obj.Cache.Pose;
        end
        
        function set.Pose(obj, pose)
            %set.Pose Set the value for property Pose
            validateattributes(pose, {obj.GeometryMsgsPoseClass}, {'nonempty', 'scalar'}, 'BoatState', 'Pose');
            
            obj.JavaMessage.setPose(pose.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Pose)
                obj.Cache.Pose.setJavaObject(pose.getJavaObject);
            end
        end
        
        function twist = get.Twist(obj)
            %get.Twist Get the value for property Twist
            if isempty(obj.Cache.Twist)
                obj.Cache.Twist = feval(obj.GeometryMsgsTwistClass, obj.JavaMessage.getTwist);
            end
            twist = obj.Cache.Twist;
        end
        
        function set.Twist(obj, twist)
            %set.Twist Set the value for property Twist
            validateattributes(twist, {obj.GeometryMsgsTwistClass}, {'nonempty', 'scalar'}, 'BoatState', 'Twist');
            
            obj.JavaMessage.setTwist(twist.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Twist)
                obj.Cache.Twist.setJavaObject(twist.getJavaObject);
            end
        end
        
        function progress = get.Progress(obj)
            %get.Progress Get the value for property Progress
            progress = double(obj.JavaMessage.getProgress);
        end
        
        function set.Progress(obj, progress)
            %set.Progress Set the value for property Progress
            validateattributes(progress, {'numeric'}, {'nonempty', 'scalar'}, 'BoatState', 'Progress');
            
            obj.JavaMessage.setProgress(progress);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Pose = [];
            obj.Cache.Twist = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Progress = obj.Progress;
            
            % Recursively copy compound properties
            cpObj.Pose = copy(obj.Pose);
            cpObj.Twist = copy(obj.Twist);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Progress = strObj.Progress;
            obj.Pose = feval([obj.GeometryMsgsPoseClass '.loadobj'], strObj.Pose);
            obj.Twist = feval([obj.GeometryMsgsTwistClass '.loadobj'], strObj.Twist);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Progress = obj.Progress;
            strObj.Pose = saveobj(obj.Pose);
            strObj.Twist = saveobj(obj.Twist);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.nautonomous_state_msgs.BoatState.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.nautonomous_state_msgs.BoatState;
            obj.reload(strObj);
        end
    end
end
