function [adjacency, vertexIndices, boundaryCells, dSi] = voronoiAdjacency(regionData, boundary)
tol1 = 1e-5;
tol2 = 1e-6;
numAgents = length(regionData);
% Allocate adjacency matrix
adjacency = zeros(numAgents,numAgents);
% Allocate boundary vector
boundaryCells = zeros(numAgents,1);
% Allocate array of indices of neighboring vertex
vertexIndices = zeros(numAgents,numAgents,2);
% Allocate cell array for indices of vertices in the boundary
dSi = cell(numAgents,1);
% Construct matrix for testing boundary membership
H = [-1  0;
    1  0;
    0 -1;
    0  1].';
boundary = [-boundary(1) boundary(2) -boundary(3) boundary(4)];
% For all but the last agent
for ii = 1:numAgents
    % Copy over data for comparing purposes
    compareListii = regionData{ii};
    if ii < numAgents
        % For possible neighbors
        for jj = ii+1:numAgents
            % Copy over data for comparing purposes
            compareListjj = regionData{jj};
            [idxboolii,idxjj] = ...
                ismembertol(compareListii, compareListjj, tol1, 'ByRows', true);
            % Check if there are two vertices that match
            if nnz(idxboolii) == 2
                % Agents are adjacent
                adjacency(ii,jj) = 1;
                adjacency(jj,ii) = 1;
                % Find vertex indices and store them
                fullListii = 1:size(compareListii, 1);
                idxii = fullListii(idxboolii);
                idxjj = idxjj(idxboolii);
                vertexIndices(ii,jj,:) = idxii(:);
                vertexIndices(jj,ii,:) = idxjj(:);
            end
        end
    end
    % Test for boundary vertices
    idxboolbound = abs(compareListii*H-boundary) < tol2;
    if nnz(idxboolbound)>0
        % Agent has vertices on the boundary
        boundaryCells(ii) = nnz(idxboolbound);
        % Find vertex indices and store them: dSi is a vector the same
        % length as the number of vertices in the cell which contains
        % the boundary id if its on the boundary or zero. The id's are:
        % 1-Left, 2-Right, 3-Down, 4-Up
        % dSi{ii} = zeros(size(compareListii,1),1);
        %[~,dSi{ii}(any(idxboolbound,2))] = find(idxboolbound,1);
        dSi{ii} = sum( cumprod(idxboolbound == 0, 2), 2) + 1;
        dSi{ii}(dSi{ii}==5) = 0;
    end
end
end