classdef matrix_helpers
    methods (Static)
        function transpose_matrix = TransposeMatrix(Phi, Theta)
            transpose_matrix = [cosd(Phi').*sind(Theta'); sind(Phi').*sind(Theta'); cosd(Theta')];
        end
    end
end
