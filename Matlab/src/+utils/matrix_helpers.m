classdef matrix_helpers
    methods (Static)
        function transpose_matrix = TransposeMatrix3d(Phi, Theta)
            transpose_matrix = [cos(Phi').*abs(sin(Theta')); sin(Phi').*abs(sin(Theta')); cos(Theta')];
        end
        function transpose_matrix = TransposeMatrix2d(Phi)
            transpose_matrix = [cos(Phi'); sin(Phi')];
        end
    end
end
