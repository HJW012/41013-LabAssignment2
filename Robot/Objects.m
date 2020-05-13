classdef Objects < handle
    properties
        % Pose of the item
        pose;
        
        % Height of the item
        height;
        
        % Various variables generated during the importing of the ply file
        % and used to transform the object throughout the environment.
        mesh_h;
        vertexCount;
        verts;
        midPoint;
        
    end
    
    methods
    
    % Given a ply file document, desired position of the object and height
    % of the object, import the object into the environment in the required
    % position
    function self = Objects(plyFile, position, height)
        % Import object from ply file
        [f, v, data] = plyread(plyFile, 'tri');                                                
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;           
            self.mesh_h = trisurf(f,v(:,1) + position(1,1) ,v(:,2) + position(1,2) ,v(:,3) + position(1,3) ...                                        
                ,'FaceVertexCData',vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
        
            % Update the number of vertices
            self.vertexCount = size(v,1);
            
            % Find the midpoint of the object
            self.midPoint = sum(v)/self.vertexCount;
            self.verts = v-repmat(self.midPoint,self.vertexCount,1);
            
            % Update the pose of the object
            self.pose = eye(4) * transl(position(1,1), position(1,2), position(1,3));
            
            % Store the height of the object
            self.height = height;
       
    end
    
    % Given a new position translate the object to that position
    function TranslateObject(self, pose)
        % Calculate the transformation matrix required to achieve the
        % desired position
        transform = inv(self.pose) * pose;
        
        % Update the pose of the object
        self.pose = self.pose * transform;
        
        % Calculate the new points of the object in the environment
        updatedPoints = [self.pose * [self.verts,ones(self.vertexCount,1)]']';  

        % Update the Vertices
        self.mesh_h.Vertices = updatedPoints(:,1:3);
        
    end
    
    % Given the end effector pose, move the object to appear as though the
    % robot is carrying the object.
    function TransformObject(self, pose)
        % Offset the desired pose by the midpoint so that it appears as
        % though the robot is carrying the object by the top
        pose = pose*transl(0,0,self.midPoint(1,3));
        
        % Calculate the transformation matrix required to transform the
        % object to the desired pose
        transform = inv(self.pose) * pose;
                       
        % Update the object's pose
        self.pose = self.pose * transform;
        
        % Calculate the updated points of the object in the environment
        updatedPoints = [self.pose * [self.verts,ones(self.vertexCount,1)]']';  

        % Update the Vertices
        self.mesh_h.Vertices = updatedPoints(:,1:3);
        
    end
    
    end
end
