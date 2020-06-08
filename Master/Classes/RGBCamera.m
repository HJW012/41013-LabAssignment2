%% Camera Class
classdef RGBCamera < handle
   properties
       centrePose = transl(0, 0, 0);
       focal = 0.08;
       pixel = 10e-5;
       resolution = [1024 1024];
       centre = [512 512];
       name = 'Generic Camera';
       
       cam;
       camHeight;
       
       object = EnvironmentObject.empty;
       objectPlot;
       fig = figure();
       figAxes = axes();
       blobPerimeters;
       blobCentroids;
       blobOrientations;
       blobBB;
       blobAreas;
       mappedCentroids;
       globalCentroids;
       globalCentroidColours = [string.empty];
       globalCentroidAreas;
       globalOrientations;
       
       im;
       BWim;
       GSim;
   end
   
   methods
       function self = RGBCamera(varargin)  
           if 0 < nargin && mod(nargin, 2) == 0
                for i = 1:2:nargin
                    knownParam = 0;
                    
                    if strcmp(varargin{i}, 'CentrePose')
                        self.centrePose = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Centre Pose set');
                    end
                    
                    if strcmp(varargin{i}, 'focal')
                        self.focal = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Focal Length set');
                    end
                    
                    if strcmp(varargin{i}, 'pixel')
                        self.pixel = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Pixel Width set');
                    end
                    
                    if strcmp(varargin{i}, 'resolution')
                        self.resolution = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Resolution set');
                    end
                    
                    if strcmp(varargin{i}, 'centre')
                        self.centre = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Image Centre set');
                    end
                    
                    if strcmp(varargin{i}, 'name')
                        self.name = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Camera Class: Camera Name set');
                    end
                    
                    if knownParam == 0
                        warning("Warning - Camera Class: Unknown Input: " + varargin{i+1});
                    end
                end
            else
                warning("Warning - Camera Class: Too Few Inputs");
            end 
            
            self.cam = CentralCamera('focal', self.focal, 'pixel', self.pixel, 'resolution', self.resolution, 'centre', self.centre, 'name', self.name);
            self.cam.T = self.centrePose;
            self.camHeight = self.centrePose(3, 4) - 0.8911; %z - table height
       end
       %% Display Camera in environment
       function DisplayCamera(self)
           self.cam.plot_camera('Tcam', self.centrePose, 'scale', 0.15);
       end
       %% Add Object to display
       function AddObject(self, object)
           self.object(numel(self.object) + 1) = object;
       end
       %% Plot objects on image plane
       function PlotObjects(self)
           
            for i = 1:numel(self.object)
                self.object(1, i).cameraPlot = self.cam.plot(self.object(1, i).modelV', 'Tobj', self.object(1, i).pose, 'MarkerSize', 0.000001, 'MarkerEdgeColor', 'white', 'MarkerFaceColor', 'white');  
                hold on;
                self.cam.hold();
            end
            
            self.fig(1) = self.cam.figure();
            set(self.fig(1), 'Resize', 'off');
            set(0, 'CurrentFigure', self.fig(1));
            self.figAxes(1) = gca(self.fig(1));
            set(self.fig(1), 'CurrentAxes', self.figAxes(1));
            hold on;
            self.cam.hold();
            
            for i = 1:numel(self.object)
                self.object(1, i).cameraK = convhull(self.object(1, i).cameraPlot');
                patch(self.figAxes(1), self.object(1, i).cameraPlot(1, self.object(1, i).cameraK), self.object(1, i).cameraPlot(2, self.object(1, i).cameraK), self.object(1, i).generalColour);
            end
       end
       %% Generate black and white image
       function ConvertImage(self)
           saveas(self.fig(1), 'image.jpg');
           
           self.im = imread('image.jpg');
           self.GSim = rgb2gray(self.im);
           self.BWim = imbinarize(self.GSim);
           self.BWim = imcomplement(self.BWim);
           self.BWim = bwareaopen(self.BWim, 100);
           se = strel('disk', 2);
           self.BWim = imclose(self.BWim, se);
           [B, L] = bwboundaries(self.BWim);
           imwrite(self.BWim, 'BWimage.jpg');
           hold on;
       end
       %% Find centroids
       function CalcBlobs(self)
           s = regionprops(self.BWim, 'centroid', 'area', 'perimeter', 'orientation', 'Extent');
           self.blobCentroids = cat(1, s.Centroid);
           self.blobPerimeters = cat(1, s.Perimeter);
           self.blobOrientations = cat(1, s.Orientation);
           self.blobBB = cat(1, s.Extent);
           self.blobAreas = cat(1, s.Area);
       end
       %% Plot Centroids
       function PlotCentroids(self)
          self.fig(2) = figure();
          
          imshow(self.BWim);
          hold on;
          
          [numRows numCols] = size(self.blobCentroids);

          for i = 1:numRows
             plot(self.blobCentroids(i, 1), self.blobCentroids(i, 2), 'b*'); 
          end
          
          self.fig(3) = figure();
          set(0, 'CurrentFigure', self.fig(3));
          imshow(self.im);
          hold on;
          for i = 1:numRows
             plot(self.blobCentroids(i, 1), self.blobCentroids(i, 2), 'y*'); 
          end 
          
          plot(self.figAxes(1), self.mappedCentroids(:, 1), self.mappedCentroids(:, 2), 'black*');
       end
       %% Map centroids to Global positions
       function MapCentroids(self)
           imInfo = imfinfo('BWimage.jpg');
           imResolution(1) = imInfo.Width;
           imResolution(2) = imInfo.Height;
           
           xLeftOffset = (185.71 / 875) * imResolution(1);
           xRightOffset = (155 / 875) * imResolution(1);
           yBottomOffset = (73 / 656) * imResolution(2);
           yTopOffset = (50 / 656) * imResolution(2);

           offsetCentroidX = self.blobCentroids(:, 1) - xLeftOffset;
           offsetCentroidY = self.blobCentroids(:, 2) - yBottomOffset;

           imageWorkspace(1) = imResolution(1) - xLeftOffset - xRightOffset; %Get axis limits within image in pixels
           imageWorkspace(2) = imResolution(2) - yBottomOffset - yTopOffset;
          
           cameraWorkspace = xlim(self.figAxes(1));
           
           self.mappedCentroids(:, 1) = (offsetCentroidX / imageWorkspace(1)) * (cameraWorkspace(2) - cameraWorkspace(1));
           self.mappedCentroids(:, 2) = (offsetCentroidY / imageWorkspace(2)) * (cameraWorkspace(2) - cameraWorkspace(1)) + (abs(yBottomOffset - yTopOffset) / imageWorkspace(2)) * (cameraWorkspace(2) - cameraWorkspace(1));
       
           posX = self.cam.T(1, 4) - ((self.mappedCentroids(:, 1) - self.cam.pp(1)) * self.camHeight) / (10000 * self.cam.f);
           posY = self.cam.T(2, 4) - ((self.mappedCentroids(:, 2) - self.cam.pp(2)) * self.camHeight) / (10000 * self.cam.f);
           pixelColours = impixel(self.im, self.blobCentroids(:, 1), self.blobCentroids(:, 2));
           [numRows numCols] = size(pixelColours);
           
           j = 1;
           for i = 1:numRows
               avg =  (pixelColours(i, 1) + pixelColours(i, 2) + pixelColours(i, 3)) / 3;
               if (avg < 200 && avg > 25 && self.blobPerimeters(i) < 300 && self.mappedCentroids(i, 1) > 0 && self.mappedCentroids(i, 2) > 0)
                   self.globalCentroids(j, 1) = posX(i);
                   self.globalCentroids(j, 2) = -1*posY(i);
                   self.globalOrientations(j) = self.blobOrientations(i);
                   self.globalCentroidAreas(j) = self.blobAreas(i);
                   if pixelColours(i, 1) >= 150
                       disp('RED OBJECT DETECTED');
                       self.globalCentroidColours(j) = 'r';
                   end
                   
                   if pixelColours(i, 2) >= 150
                       disp('YELLOW OBJECT DETECTED');
                       self.globalCentroidColours(j) = 'g';
                   end
                   
                   if pixelColours(i, 3) >= 100
                       disp('BLUE OBJECT DETECTED');
                       self.globalCentroidColours(j) = 'b';
                   end
                   
                   j = j + 1;
               end
           end
       end
       %% Detect Centroids
       function LocateObjects(self)
           self.PlotObjects();
           self.ConvertImage();
           self.CalcBlobs();
           self.MapCentroids();
           %self.PlotCentroids();
       end
   end
end