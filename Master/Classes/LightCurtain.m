%% Light Curtain Class
classdef LightCurtain < handle
   properties
        centre;
        lightCurtain;
        curtain_h;
        tableWidth;
        tableLength;
        tableHeight;
        model = EnvironmentObject.empty;
   end
   
   methods 
       function self = LightCurtain(table)
           
            self.centre = [0,0,0];
            self.tableWidth = table.dimensions(1,1);
            self.tableLength = table.dimensions(1,2);
            self.tableHeight = table.dimensions(1,3);
            self.model = EnvironmentObject('Type', 'safety', 'ModelPath', 'lightCurtain.ply', 'Pose', transl(0, 0, self.tableHeight));
            self.GetLightCurtain();
       end
       
       function GetLightCurtain(self)
            x = (self.centre(1,1) - (0.5*self.tableWidth) - 0.05);
            z = (self.tableHeight + 0.8);

            self.lightCurtain = [];

            for y = (self.centre(1,2) - (0.5*self.tableLength)):0.1:(self.centre(1,2) + (0.5*self.tableLength))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            x = (self.centre(1,1) + (0.5*self.tableWidth) + 0.05);
            for y = (self.centre(1,2) - (0.5*self.tableLength)):0.1:(self.centre(1,2) + (0.5*self.tableLength))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            y = (self.centre(1,2) - (0.5*self.tableLength) - 0.07);
            for x = (self.centre(1,1) - (0.5*self.tableWidth)):0.1:(self.centre(1,1) + (0.5*self.tableWidth))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end

            y = (self.centre(1,2) + (0.5*self.tableLength) + 0.02);
            for x = (self.centre(1,1) - (0.5*self.tableWidth)):0.1:(self.centre(1,1) + (0.5*self.tableWidth))
                self.lightCurtain = [self.lightCurtain; x, y, self.tableHeight, x, y, z];
            end
       end
       
       function Display(self)
           self.model.Display();
       end
       
       
       function checkCurtain = CheckLightCurtain(self, object)
            checkCurtain = 0;
            
            [k, volume] = convhull(object.modelMesh.Vertices(:, 1), object.modelMesh.Vertices(:, 2), object.modelMesh.Vertices(:, 3)); 
            tr = triangulation(k, object.modelMesh.Vertices(:, 1), object.modelMesh.Vertices(:, 2), object.modelMesh.Vertices(:, 3)); 
            tempModelF = tr.ConnectivityList;
            tempModelV = tr.Points;
            faceNormals = zeros(size(tempModelF,1),3);

            for faceIndex = 1:size(tempModelF,1)
                v1 = tempModelV(tempModelF(faceIndex,1)',:);
                v2 = tempModelV(tempModelF(faceIndex,2)',:);
                v3 = tempModelV(tempModelF(faceIndex,3)',:);
                faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            for i = 1:1:size(self.lightCurtain, 1)
                point1 = [self.lightCurtain(i,1), self.lightCurtain(i,2), self.lightCurtain(i,3)];
                point2 = [self.lightCurtain(i,4), self.lightCurtain(i,5), self.lightCurtain(i,6)];
                
                for faceIndex = 1:size(tempModelF,1)
                    vertOnPlane = tempModelV(tempModelF(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,point1,point2); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,tempModelV(tempModelF(faceIndex,:)',:))
                        checkCurtain = 1;
                        break;
                    end
                end
                
                if 0 < checkCurtain
                    break;
                end
                
            end
       end
   end
end