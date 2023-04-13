classdef matrix_helpers
    methods (Static)
        function tarnsposed_matrix = TransposeMatrix(Phi, Theta)
            tarnsposed_matrix = [cosd(Phi).*sind(Theta); sind(Phi).*sind(Theta); cosd(Theta)]; 
        end
    end
end
