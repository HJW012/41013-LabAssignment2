%% Object class
classdef RemoteController < handle
   properties
       ID = 2;
       State = false;
   end
   
   methods
       function self = RemoteController(varargin)  
           if 0 < nargin && mod(nargin, 2) == 0
              for i = 1:2:nargin
                  knownParam = 0;
                  
                  if strcmp(varargin{i}, 'ID')
                      [self.modelF, self.modelV, self.modelData] = plyread(varargin{i+1}, "tri");
                      self.vertexColours = [self.modelData.vertex.red, self.modelData.vertex.green, self.modelData.vertex.blue] / 255;
                      self.vertexCount = size(self.modelV, 1);
                      knownParam = 1;
                  end
                  
                  if knownParam == 0
                     warning("Unknown Param: " + varargin{i+1});
                  end
              end
              
           else
               warning("Warning - Object Class: Too Few Inputs");
           end
       end
   end
  
end