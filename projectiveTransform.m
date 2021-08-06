function [xc,varargout] = projectiveTransform(x,H,varargin)
%PROJECTIVETRANSFORM Computes the projective transformation of points x.
%Can also be used to compute the jacobian of the transformation and the
%velocity transformation.
%   Input arguments
%       - x: 2-by-m array of positions
%       - H: 3-by-3 homogeneous coordinate transformation parameters
%       - v: 2-by-m array of velocities at x
%   Output arguments
%       - xc: 2-by-m array of transformed positions
%       - vc: 2-by-m array of transformed velocities at x
%       - Jc: 2-by-2-by-m array of Jacobian matrices at x

narginchk(2,3);
nargoutchk(0,3);
if nargin > 2
    v = varargin{1};
    assert(size(v) == size(x), 'The velocity array must have the same dimensions as the position array.')
else
    v = nan(size(x));
end

assert(size(x,1)==2,'The position array must be a 2-by-m matrix.')

xcnum = bsxfun( @plus , H(1:2,1:2)*x , H(1:2,3) );
xcden = H(3,1:2)*x + H(3,3);
xc = bsxfun( @rdivide , xcnum , xcden );

%%TODO: Compute vc and Jc
% dnum = H(1:2,1:2);
% dden = H(3,1:2);
% 
% Jc = (dnum*xcden-xcnum*dden)./xcden.^2;
Jc = nan(2,2,size(x,2));
vc = v;

if nargout > 1
    varargout{1} = vc;
    if nargout > 2
       varargout{2} = Jc; 
    end
end
end

