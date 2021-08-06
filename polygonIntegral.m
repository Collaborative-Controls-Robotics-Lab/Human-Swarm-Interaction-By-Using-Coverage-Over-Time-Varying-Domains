function [integralApprox] = polygonIntegral(func,vertices,xmesh,ymesh)
%POLYGONINTEGRAL Approximates the surface integral of a function func
%evaluated over a polygonal region with a vertex set vertices, oriented cw.

% Get bounds of polygon
xmin = min(vertices(:,1));
xmax = max(vertices(:,1));
ymin = min(vertices(:,2));
ymax = max(vertices(:,2));

% Scale and shift the mesh to snuggly fit the polygon
xmesh = (xmax-xmin)*xmesh + xmin;
ymesh = (ymax-ymin)*ymesh + ymin;

% Get an area element
dxy = diff(xmesh(1,1:2))*diff(ymesh(1:2,1));

% Get centroid of each cell
dc = zeros([size(xmesh)-1,2]);
dc(:,:,1) = 0.5*(xmesh(1:end-1,1:end-1)+xmesh(2:end,2:end));
dc(:,:,2) = 0.5*(ymesh(1:end-1,1:end-1)+ymesh(2:end,2:end));
dcVec = [reshape(dc(:,:,1),[],1),reshape(dc(:,:,2),[],1)];

% Find the centroids inside the polygon
centroidsInPoly = inpolygon(dcVec(:,1),dcVec(:,2),vertices(:,1),vertices(:,2));

% Evaluate the integrand at the centroid values to form volume cubes
integrand = func(dcVec(centroidsInPoly,1),dcVec(centroidsInPoly,2));

% Add up volume of the cubes inside polygon
integralApprox = sum(integrand*dxy);

end

