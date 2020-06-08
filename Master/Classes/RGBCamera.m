%% Camera Class
classdef RGBCamera < handle
   properties
       
       % Internal parameters for Peter Corke Camera
       % These are default values, can be changed in constructor
       centrePose = transl(0, 0, 0);
       focal = 0.08;
       pixel = 10e-5;
       resolution = [1024 1024];
       centre = [512 512];
       name = 'Desk Organiser Camera';
       cam;
       
       %Height of camera above table
       camHeight;
       
       % Object matrix to display in camera plane
       object = EnvironmentObject.empty;
       
       % Stores all relevant figures 
       % - Camera plot
       % - Black and white image figure
       % - Colour image figure
       fig = figure();
       
       % Stores axes on camera plot - to plot centroids later on
       figAxes = axes();
       
       % Blob data storage for mapping/manipulation
       blobPerimeters;
       blobCentroids;
       blobOrientations;
       blobBB;
       blobAreas;
       
       % Mapped centroid coordinates calculated to plot on camera plot
       mappedCentroids;
       
       % Global blob data to be used by global controller
       globalCentroids;
       globalCentroidColours = [string.empty];
       globalCentroidAreas;
       globalOrientations;
       
       % Stores images taken of camera plot figure
       im;
       GSim;
       BWim;
   end
   
   methods
       function self = RGBCamera(varargin)  
           % Accepts variable inputs - if some parameters not input,
           % defaults will be used
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
                %warning("Warning - Camera Class: Too Few Inputs");
           end 
            
            % Creates Peter Corke camera object with camera parameters
            self.cam = CentralCamera('focal', self.focal, 'pixel', self.pixel, 'resolution', self.resolution, 'centre', self.centre, 'name', self.name);
            self.cam.T = self.centrePose;
            
            % Cam height used for converting mapped centroid coordinates to
            % global coordinates
            self.camHeight = self.centrePose(3, 4) - 0.8911; %z - table height
       end
       %% DisplayCamera - Displays camera in environment
       function DisplayCamera(self)
           self.cam.plot_camera('Tcam', self.centrePose, 'scale', 0.15); % Scale 0.15 for large camera - does not affect camera functionality
       end
       %% AddObject - Add Object to be plotted in camera plane
       function AddObject(self, object)
           self.object(numel(self.object) + 1) = object;
       end
       %% PlotObjects - Plot objects on camera plane
       function PlotObjects(self)
            % Cycle through all objects to be displayed, and plot on camera
            % plane
            for i = 1:numel(self.object)
                % Plots all model vertices of each object object
                %  - Tobj used instead of Tcam to keep camera stationary and
                %    to plot objects in their respective locations
                %  - MarkerSize set low to avoid issues when detecting
                %    blobs
                %  - Same reason for white marker face/edge colour
                self.object(1, i).cameraPlot = self.cam.plot(self.object(1, i).modelV', 'Tobj', self.object(1, i).pose, 'MarkerSize', 0.000001, 'MarkerEdgeColor', 'white', 'MarkerFaceColor', 'white');  
                hold on;
                self.cam.hold();
            end
            
            % Stores camera plane figure for later plotting
            self.fig(1) = self.cam.figure();
            
            % Prevent resizing in camera plane to avoid any image
            % resolution issues
            set(self.fig(1), 'Resize', 'off');
            
            % Just to be safe, make sure the camera plane is in focus
            set(0, 'CurrentFigure', self.fig(1));
            
            % Gets and sets camera plane axes for later plotting
            self.figAxes(1) = gca(self.fig(1));
            set(self.fig(1), 'CurrentAxes', self.figAxes(1));
            
            hold on;
            self.cam.hold();
            
            for i = 1:numel(self.object)
                % Get just the outside points from object plots on camera
                % plane - NOT FROM OBJECT MODEL VERTICES
                self.object(1, i).cameraK = convhull(self.object(1, i).cameraPlot');
                
                % Fill these outside points on the camera plane axes, X and
                % Y values of outside points, and general colour provided
                % in each environment object constructor
                patch(self.figAxes(1), self.object(1, i).cameraPlot(1, self.object(1, i).cameraK), self.object(1, i).cameraPlot(2, self.object(1, i).cameraK), self.object(1, i).generalColour);
            end
       end
       %% ConvertImage - Generate black and white & grayscale images
       function ConvertImage(self)
           % Save camera plane figure to image
           saveas(self.fig(1), 'image.jpg');
   
           % Read image back into workspace
           self.im = imread('image.jpg');
           
           % Convert to grayscale - required for imbinarize
           self.GSim = rgb2gray(self.im);
           
           % Convert grayscale to black and white
           self.BWim = imbinarize(self.GSim);
           
           %Invert image to get blobs as white instead of black
           self.BWim = imcomplement(self.BWim);
           
           % Remove blobs with less than 100 pixels
           self.BWim = bwareaopen(self.BWim, 100);      
           
           % Encircle blobs - also also not overly useful here
           [B, L] = bwboundaries(self.BWim);
           
           % Write these blobs to image
           imwrite(self.BWim, 'BWimage.jpg');
           
           
           hold on;
       end
       %% CalcBlobs - Detect blob properties in image
       function CalcBlobs(self)
           % Detect properties of image regions
           s = regionprops(self.BWim, 'centroid', 'area', 'perimeter', 'orientation', 'Extent');
           
           % Store all relevant blob info
           self.blobCentroids = cat(1, s.Centroid);
           self.blobPerimeters = cat(1, s.Perimeter);
           self.blobOrientations = cat(1, s.Orientation);
           self.blobBB = cat(1, s.Extent);
           self.blobAreas = cat(1, s.Area);
       end
       %% PlotCentroids - Plot centroids on images/camera plane
       function PlotCentroids(self)
          % Create new figure for black and white image
          self.fig(2) = figure();
          set(0, 'CurrentFigure', self.fig(2));
          imshow(self.BWim);
          hold on;
          
          % Plot detected centroids on BW image
          [numRows numCols] = size(self.blobCentroids);
          for i = 1:numRows
             plot(self.blobCentroids(i, 1), self.blobCentroids(i, 2), 'b*'); 
          end
          
          % Create new figure for colour image
          self.fig(3) = figure();
          set(0, 'CurrentFigure', self.fig(3));
          imshow(self.im);
          hold on;
          
          % Plot detected centroids on colour image
          for i = 1:numRows
             plot(self.blobCentroids(i, 1), self.blobCentroids(i, 2), 'y*'); 
          end 
          
          % Plot mapped centroid values (calculated below) on original
          % camer plane using the stored axes
          plot(self.figAxes(1), self.mappedCentroids(:, 1), self.mappedCentroids(:, 2), 'black*');
       end
       %% MapCentroids - Map centroids to Global positions
       function MapCentroids(self)
           % Read resolution from from image - this is to cater for other
           % people's screen resolutions possibly
           imInfo = imfinfo('BWimage.jpg');
           imResolution(1) = imInfo.Width;
           imResolution(2) = imInfo.Height;
           
           % Measured in photoshop - offsets between axes and image sides
           % then applied to possibly different image resolution
           xLeftOffset = (185.71 / 875) * imResolution(1);
           xRightOffset = (155 / 875) * imResolution(1);
           yBottomOffset = (73 / 656) * imResolution(2);
           yTopOffset = (50 / 656) * imResolution(2);

           % Shift centroids over by these offsets - required before
           % mapping to camera plane axes
           offsetCentroidX = self.blobCentroids(:, 1) - xLeftOffset;
           offsetCentroidY = self.blobCentroids(:, 2) - yBottomOffset;

           % Get axis limits within image in pixels
           imageWorkspace(1) = imResolution(1) - xLeftOffset - xRightOffset;
           imageWorkspace(2) = imResolution(2) - yBottomOffset - yTopOffset;
          
           % Get axis limits on camer plane - not really necessary since
           % camera resolution is stored internally on camera. returns
           % lower and upper limits (0, 1024)
           cameraWorkspace = xlim(self.figAxes(1));
           
           % Map centroid image coordinates to camera axes by taking ratio
           % of coordinate to images axes size in pixels, then applying to
           % camera plane axes (1024 x 1024)
           self.mappedCentroids(:, 1) = (offsetCentroidX / imageWorkspace(1)) * (cameraWorkspace(2) - cameraWorkspace(1));
           
           % Map centroid Y values - trickier than because image
           % coordinates are from top left corner, camera plane coordinates
           % are from bottom left corner of axes
           self.mappedCentroids(:, 2) = (offsetCentroidY / imageWorkspace(2)) * (cameraWorkspace(2) - cameraWorkspace(1)) + (abs(yBottomOffset - yTopOffset) / imageWorkspace(2)) * (cameraWorkspace(2) - cameraWorkspace(1));
       
           % Calculate global centroid position using known camera height
           % and formula:
           % X = cam(x)-((image(X)-principlePoint) * height) / focal length
           posX = self.cam.T(1, 4) - ((self.mappedCentroids(:, 1) - self.cam.pp(1)) * self.camHeight) / (10000 * self.cam.f);
           posY = self.cam.T(2, 4) - ((self.mappedCentroids(:, 2) - self.cam.pp(2)) * self.camHeight) / (10000 * self.cam.f);
           pixelColours = impixel(self.im, self.blobCentroids(:, 1), self.blobCentroids(:, 2));
           [numRows numCols] = size(pixelColours);
           
           % Cycle through each centroid and filter out by colour and by camera plot Y coords:
           % - Average R G B values
           % - High value (>200) would be white - delete
           % - Low value (<20) would be balck - delete
           j = 1;
           for i = 1:numRows
               avg =  (pixelColours(i, 1) + pixelColours(i, 2) + pixelColours(i, 3)) / 3;
               if (avg < 200 && avg > 25 && self.blobPerimeters(i) < 300 && self.mappedCentroids(i, 1) > 0 && self.mappedCentroids(i, 2) > 0)
                   self.globalCentroids(j, 1) = posX(i);
                   self.globalCentroids(j, 2) = -1*posY(i); % Camera pose is rotates, so must invert Y vals
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
                   
                   if pixelColours(i, 3) >= 100 % Had issues with higher tolerance for blue
                       disp('BLUE OBJECT DETECTED');
                       self.globalCentroidColours(j) = 'b';
                   end
                   
                   j = j + 1;
               end
           end
       end
       %% LocateObjects - Run all other functions to fine target objects
       function LocateObjects(self)
           self.PlotObjects();
           self.ConvertImage();
           self.CalcBlobs();
           self.MapCentroids();
           self.PlotCentroids();
       end
   end
end