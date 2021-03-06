%% Light Curtain Class
classdef LightCurtain < handle
   properties
        centre;                             % Centre of the light curtain
        lightCurtain;                       % Array to store the points of the light curtain
        tableWidth;                         % Width of the table around which the light curtain will exist
        tableLength;                        % Length of the table around which the light curtain will exist
        tableHeight;                        % Height of table around which the light curtain will exist
        model = EnvironmentObject.empty;    % Model used to visualise the light curtain
   end
   
   methods 
       % Constructor used to initialise the properties of the class
       function self = LightCurtain(table)
            % Initialise all properties of the class
            self.centre = [0,0,0];
            self.tableWidth = table.dimensions(1,1);
            self.tableLength = table.dimensions(1,2);
            self.tableHeight = table.dimensions(1,3);
            self.model = EnvironmentObject('Type', 'safety', 'ModelPath', 'lightCurtain.ply', 'Pose', transl(0, 0, self.tableHeight));
            
            % Call function to generate the light curtain
            self.GetLightCurtain();
       end
       %% GetLightCurtain
       % Function used to generate the points which make up the light
       % curtain
       function GetLightCurtain(self)
            % Set initial x position to be on the lower side of the table
            % width. Offset applied to try and level the position of the
            % light curtain with the table
            x = (self.centre(1,1) - (0.5*self.tableWidth) - 0.05);
            
            % Set top of light curtain to be 0.8 m above the surface of the
            % table
            z = (self.tableHeight + 0.8);

            % Initialise the light curtain array
            self.lightCurtain = [];

            % Iterate through the y positions every 0.1m along the length
            % of the table and create a pair of points, one at table height
            % and the other at 0.8 m above table surface
            for y = (self.centre(1,2) - (0.5*self.tableLength)):0.1:(self.centre(1,2) + (0.5*self.tableLength))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            % Set new x position at upper limit of table width
            x = (self.centre(1,1) + (0.5*self.tableWidth) + 0.05);
            
            % Iterate through the y positions every 0.1m along the length
            % of the table and create a pair of points, one at table height
            % and the other at 0.8 m above table surface
            for y = (self.centre(1,2) - (0.5*self.tableLength)):0.1:(self.centre(1,2) + (0.5*self.tableLength))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            % Set y position to be at lower limit of table length
            y = (self.centre(1,2) - (0.5*self.tableLength) - 0.07);
            
            % Iterate through the x positions every 0.1m along the width
            % of the table and create a pair of points, one at table height
            % and the other at 0.8 m above table surface
            for x = (self.centre(1,1) - (0.5*self.tableWidth)):0.1:(self.centre(1,1) + (0.5*self.tableWidth))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            % Set y position to be at upper limit of table length
            y = (self.centre(1,2) + (0.5*self.tableLength) + 0.02);
            
            % Iterate through the x positions every 0.1m along the width
            % of the table and create a pair of points, one at table height
            % and the other at 0.8 m above table surface
            for x = (self.centre(1,1) - (0.5*self.tableWidth)):0.1:(self.centre(1,1) + (0.5*self.tableWidth))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end
       end
       
       %% Display
       % Display ply model used to visualise the light curtain
       function Display(self)
           self.model.Display();
       end
       
       %% CheckLightCurtain
       % Function used to check if an object breaks the light curtain
       function checkCurtain = CheckLightCurtain(self, object)
            % Initialise flag to say object does not obstruct light curtain
            checkCurtain = 0;
            
            % Turn warnings off to avoid flooding command prompt with warnings regarding triangulation
            warning off;
            
            % Collect just the outside vertices of the object being checked
            [k, volume] = convhull(object.modelMesh.Vertices(:, 1), object.modelMesh.Vertices(:, 2), object.modelMesh.Vertices(:, 3)); 
            
            % Use triangulation to generate faces from the outside vertices
            tr = triangulation(k, object.modelMesh.Vertices(:, 1), object.modelMesh.Vertices(:, 2), object.modelMesh.Vertices(:, 3)); 
            tempModelF = tr.ConnectivityList;
            tempModelV = tr.Points;
            
            % Initialise face normals matrix
            faceNormals = zeros(size(tempModelF,1),3);

            % Iterate through each face of the object's convhulled vertices
            % and store the three vertices which create the face
            for faceIndex = 1:size(tempModelF,1)
                v1 = tempModelV(tempModelF(faceIndex,1)',:);
                v2 = tempModelV(tempModelF(faceIndex,2)',:);
                v3 = tempModelV(tempModelF(faceIndex,3)',:);
                
                % Using cross product find the normal of the face
                faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            % Iterate through every 'line' create by the pair of points in
            % the light curtain and check if the line between these two
            % points intersects any of the faces of the object
            for i = 1:1:size(self.lightCurtain, 1)
                % Store paired points in light curtain
                point1 = [self.lightCurtain(i,1), self.lightCurtain(i,2), self.lightCurtain(i,3)];
                point2 = [self.lightCurtain(i,4), self.lightCurtain(i,5), self.lightCurtain(i,6)];
                
                % Iterate through each face in the object
                for faceIndex = 1:size(tempModelF,1)
                    % Find a vertex on the plane of the object's face
                    vertOnPlane = tempModelV(tempModelF(faceIndex,1)',:);
                    
                    % Check if the line between the two points intersects
                    % with the face of the object
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,point1,point2); 
                    
                    % If the line does intersect and also passes through
                    % the face of the created by the three vertices on the
                    % object, set the flag to 1 and break out of the for
                    % loop
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,tempModelV(tempModelF(faceIndex,:)',:))
                        checkCurtain = 1;
                        break;
                    end
                end
                
                % If the flag has been set, break out of the for loop
                if 0 < checkCurtain
                    break;
                end
            end
            warning on;
       end
   end
end
