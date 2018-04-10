classdef BoatParam < robotics.ros.Message
    %BoatParam MATLAB implementation of nautonomous_state_msgs/BoatParam
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'nautonomous_state_msgs/BoatParam' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '12618c8c534ba6d1eb6d2ca43089aefb' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Length
        Width
        SafetyRadius
        M
        IZ
        L
        TFull
        THalf
        Beta
        DX
        DY
        DTheta
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Beta', 'DTheta', 'DX', 'DY', 'IZ', 'L', 'Length', 'M', 'SafetyRadius', 'TFull', 'THalf', 'Width'} % List of non-constant message properties
        ROSPropertyList = {'beta', 'D_theta', 'D_x', 'D_y', 'I_z', 'l', 'length', 'm', 'safety_radius', 'T_full', 'T_half', 'width'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = BoatParam(msg)
            %BoatParam Construct the message object BoatParam
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
        
        function length = get.Length(obj)
            %get.Length Get the value for property Length
            length = double(obj.JavaMessage.getLength);
        end
        
        function set.Length(obj, length)
            %set.Length Set the value for property Length
            validateattributes(length, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'Length');
            
            obj.JavaMessage.setLength(length);
        end
        
        function width = get.Width(obj)
            %get.Width Get the value for property Width
            width = double(obj.JavaMessage.getWidth);
        end
        
        function set.Width(obj, width)
            %set.Width Set the value for property Width
            validateattributes(width, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'Width');
            
            obj.JavaMessage.setWidth(width);
        end
        
        function safetyradius = get.SafetyRadius(obj)
            %get.SafetyRadius Get the value for property SafetyRadius
            safetyradius = double(obj.JavaMessage.getSafetyRadius);
        end
        
        function set.SafetyRadius(obj, safetyradius)
            %set.SafetyRadius Set the value for property SafetyRadius
            validateattributes(safetyradius, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'SafetyRadius');
            
            obj.JavaMessage.setSafetyRadius(safetyradius);
        end
        
        function m = get.M(obj)
            %get.M Get the value for property M
            m = double(obj.JavaMessage.getM);
        end
        
        function set.M(obj, m)
            %set.M Set the value for property M
            validateattributes(m, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'M');
            
            obj.JavaMessage.setM(m);
        end
        
        function iz = get.IZ(obj)
            %get.IZ Get the value for property IZ
            iz = double(obj.JavaMessage.getIZ);
        end
        
        function set.IZ(obj, iz)
            %set.IZ Set the value for property IZ
            validateattributes(iz, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'IZ');
            
            obj.JavaMessage.setIZ(iz);
        end
        
        function l = get.L(obj)
            %get.L Get the value for property L
            l = double(obj.JavaMessage.getL);
        end
        
        function set.L(obj, l)
            %set.L Set the value for property L
            validateattributes(l, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'L');
            
            obj.JavaMessage.setL(l);
        end
        
        function tfull = get.TFull(obj)
            %get.TFull Get the value for property TFull
            tfull = double(obj.JavaMessage.getTFull);
        end
        
        function set.TFull(obj, tfull)
            %set.TFull Set the value for property TFull
            validateattributes(tfull, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'TFull');
            
            obj.JavaMessage.setTFull(tfull);
        end
        
        function thalf = get.THalf(obj)
            %get.THalf Get the value for property THalf
            thalf = double(obj.JavaMessage.getTHalf);
        end
        
        function set.THalf(obj, thalf)
            %set.THalf Set the value for property THalf
            validateattributes(thalf, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'THalf');
            
            obj.JavaMessage.setTHalf(thalf);
        end
        
        function beta = get.Beta(obj)
            %get.Beta Get the value for property Beta
            beta = double(obj.JavaMessage.getBeta);
        end
        
        function set.Beta(obj, beta)
            %set.Beta Set the value for property Beta
            validateattributes(beta, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'Beta');
            
            obj.JavaMessage.setBeta(beta);
        end
        
        function dx = get.DX(obj)
            %get.DX Get the value for property DX
            dx = double(obj.JavaMessage.getDX);
        end
        
        function set.DX(obj, dx)
            %set.DX Set the value for property DX
            validateattributes(dx, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'DX');
            
            obj.JavaMessage.setDX(dx);
        end
        
        function dy = get.DY(obj)
            %get.DY Get the value for property DY
            dy = double(obj.JavaMessage.getDY);
        end
        
        function set.DY(obj, dy)
            %set.DY Set the value for property DY
            validateattributes(dy, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'DY');
            
            obj.JavaMessage.setDY(dy);
        end
        
        function dtheta = get.DTheta(obj)
            %get.DTheta Get the value for property DTheta
            dtheta = double(obj.JavaMessage.getDTheta);
        end
        
        function set.DTheta(obj, dtheta)
            %set.DTheta Set the value for property DTheta
            validateattributes(dtheta, {'numeric'}, {'nonempty', 'scalar'}, 'BoatParam', 'DTheta');
            
            obj.JavaMessage.setDTheta(dtheta);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Length = obj.Length;
            cpObj.Width = obj.Width;
            cpObj.SafetyRadius = obj.SafetyRadius;
            cpObj.M = obj.M;
            cpObj.IZ = obj.IZ;
            cpObj.L = obj.L;
            cpObj.TFull = obj.TFull;
            cpObj.THalf = obj.THalf;
            cpObj.Beta = obj.Beta;
            cpObj.DX = obj.DX;
            cpObj.DY = obj.DY;
            cpObj.DTheta = obj.DTheta;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Length = strObj.Length;
            obj.Width = strObj.Width;
            obj.SafetyRadius = strObj.SafetyRadius;
            obj.M = strObj.M;
            obj.IZ = strObj.IZ;
            obj.L = strObj.L;
            obj.TFull = strObj.TFull;
            obj.THalf = strObj.THalf;
            obj.Beta = strObj.Beta;
            obj.DX = strObj.DX;
            obj.DY = strObj.DY;
            obj.DTheta = strObj.DTheta;
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
            
            strObj.Length = obj.Length;
            strObj.Width = obj.Width;
            strObj.SafetyRadius = obj.SafetyRadius;
            strObj.M = obj.M;
            strObj.IZ = obj.IZ;
            strObj.L = obj.L;
            strObj.TFull = obj.TFull;
            strObj.THalf = obj.THalf;
            strObj.Beta = obj.Beta;
            strObj.DX = obj.DX;
            strObj.DY = obj.DY;
            strObj.DTheta = obj.DTheta;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.nautonomous_state_msgs.BoatParam.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.nautonomous_state_msgs.BoatParam;
            obj.reload(strObj);
        end
    end
end
