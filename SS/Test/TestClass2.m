%% Object class
classdef TestClass2 < handle
   properties
      State = 0;
      buffer = 0.05;
   end
   events
      ToggledState
   end
   methods
      function OnStateChange(obj,newState)
         if abs(newState - obj.State) > obj.buffer
            obj.State = newState;
            notify(obj,'ToggledState');
         end
      end
   end
end