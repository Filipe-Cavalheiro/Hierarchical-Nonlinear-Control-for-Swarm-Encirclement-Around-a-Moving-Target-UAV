% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function x=unskew(X)
% Obtain skew-symmetric matrix from vector

n = size(X,1);
if n == 3
    x = [X(3,2);X(1,3);X(2,1)];
elseif n == 1
    x = X(2,1);
else
    error('UNSKEW function not implemented for input dimensions other than 1 or 3 (i.e., so(2) and so(3)).');
end