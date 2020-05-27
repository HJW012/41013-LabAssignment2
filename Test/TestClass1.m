%% Object class
classdef TestClass1 < handle
   methods
      function obj = TestClass1(toggle_button_obj)
         addlistener(toggle_button_obj,'ToggledState',@TestClass1.handleEvnt);
      end
   end
   methods (Static)
      function handleEvnt(src,~)
         disp(src.State);
      end
   end
end