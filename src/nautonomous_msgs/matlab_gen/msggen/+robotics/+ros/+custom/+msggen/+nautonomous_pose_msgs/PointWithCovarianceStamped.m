classdef PointWithCovarianceStamped < robotics.ros.Message
    %PointWithCovarianceStamped MATLAB implementation of nautonomous_pose_msgs/PointWithCovarianceStamped
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'nautonomous_pose_msgs/PointWithCovarianceStamped' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '8e7cbc07d5a4cc01247678d357ac7cae' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant)
        COVARIANCETYPEUNKNOWN = uint8(0)
        COVARIANCETYPEAPPROXIMATED = uint8(1)
        COVARIANCETYPEDIAGONALKNOWN = uint8(2)
        COVARIANCETYPEKNOWN = uint8(3)
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPointClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Point') % Dispatch to MATLAB class for message type geometry_msgs/Point
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Point
        PositionCovarianceType
        PositionCovariance
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Point', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Header', 'Point', 'PositionCovariance', 'PositionCovarianceType'} % List of non-constant message properties
        ROSPropertyList = {'header', 'point', 'position_covariance', 'position_covariance_type'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = PointWithCovarianceStamped(msg)
            %PointWithCovarianceStamped Construct the message object PointWithCovarianceStamped
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
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'PointWithCovarianceStamped', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function point = get.Point(obj)
            %get.Point Get the value for property Point
            if isempty(obj.Cache.Point)
                obj.Cache.Point = feval(obj.GeometryMsgsPointClass, obj.JavaMessage.getPoint);
            end
            point = obj.Cache.Point;
        end
        
        function set.Point(obj, point)
            %set.Point Set the value for property Point
            validateattributes(point, {obj.GeometryMsgsPointClass}, {'nonempty', 'scalar'}, 'PointWithCovarianceStamped', 'Point');
            
            obj.JavaMessage.setPoint(point.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Point)
                obj.Cache.Point.setJavaObject(point.getJavaObject);
            end
        end
        
        function positioncovariancetype = get.PositionCovarianceType(obj)
            %get.PositionCovarianceType Get the value for property PositionCovarianceType
            positioncovariancetype = typecast(int8(obj.JavaMessage.getPositionCovarianceType), 'uint8');
        end
        
        function set.PositionCovarianceType(obj, positioncovariancetype)
            %set.PositionCovarianceType Set the value for property PositionCovarianceType
            validateattributes(positioncovariancetype, {'numeric'}, {'nonempty', 'scalar'}, 'PointWithCovarianceStamped', 'PositionCovarianceType');
            
            obj.JavaMessage.setPositionCovarianceType(positioncovariancetype);
        end
        
        function positioncovariance = get.PositionCovariance(obj)
            %get.PositionCovariance Get the value for property PositionCovariance
            javaArray = obj.JavaMessage.getPositionCovariance;
            array = obj.readJavaArray(javaArray, 'double');
            positioncovariance = double(array);
        end
        
        function set.PositionCovariance(obj, positioncovariance)
            %set.PositionCovariance Set the value for property PositionCovariance
            validateattributes(positioncovariance, {'numeric'}, {'vector', 'numel', 9}, 'PointWithCovarianceStamped', 'PositionCovariance');
            
            javaArray = obj.JavaMessage.getPositionCovariance;
            array = obj.writeJavaArray(positioncovariance, javaArray, 'double');
            obj.JavaMessage.setPositionCovariance(array);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Point = [];
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
            cpObj.PositionCovarianceType = obj.PositionCovarianceType;
            cpObj.PositionCovariance = obj.PositionCovariance;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.Point = copy(obj.Point);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.PositionCovarianceType = strObj.PositionCovarianceType;
            obj.PositionCovariance = strObj.PositionCovariance;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Point = feval([obj.GeometryMsgsPointClass '.loadobj'], strObj.Point);
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
            
            strObj.PositionCovarianceType = obj.PositionCovarianceType;
            strObj.PositionCovariance = obj.PositionCovariance;
            strObj.Header = saveobj(obj.Header);
            strObj.Point = saveobj(obj.Point);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.nautonomous_pose_msgs.PointWithCovarianceStamped.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.nautonomous_pose_msgs.PointWithCovarianceStamped;
            obj.reload(strObj);
        end
    end
end
