function X = poly2poly(P1, P2)
% function X = poly2poly(P1, P2)
% Intersection of two 2D polygons P1 and P2.
%
% INPUTS:
%   P1 and P2 are two-row arrays, each column is a vertice
%   They might or might not be wrapped around
% OUTPUT:
%   X is two-row array, each column is an intersecting point
%
% Author: Bruno Luong <brunoluong@yahoo.com>
% History:
%     Original 20-May-2010

% Wrap around: Pad the first point to the end if necessary
if ~isequal(P1(:,1),P1(:,end))
    P1 = [P1 P1(:,1)];
end
if ~isequal(P2(:,1),P2(:,end))
    P2 = [P2 P2(:,1)];
end

% swap P1 P2 so that we loop on a smaller one
if size(P1,2) > size(P2,2)
    [P1 P2] = deal(P2, P1);
end

% We increment the intermediate results by this amount
increment = 10;
% Empty buffer
X = zeros(2,0);
filled = 0;
sizec = 0;
% Loop over segments of P1
for n=2:size(P1,2)
    cn = seg2poly(P1(:,n-1:n), P2);
    m = size(cn,2);
    filled = filled+m;
    % Buffer too small
    if sizec < filled
        sizec = filled+increment;
        X(2,sizec) = 0;
    end
    % Store the result
    X(:,filled+(-m+1:0)) = cn;
end
% remove the tail
X(:,filled+1:end) = [];

end % poly2poly
