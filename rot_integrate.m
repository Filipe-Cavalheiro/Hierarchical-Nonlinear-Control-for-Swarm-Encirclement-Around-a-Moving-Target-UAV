% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function Rp = rot_integrate(R,om,dT)
    
    nsteps = 10;
    Rp = R;
    for i = 1:nsteps
        Rp = Rp*expm(dT/nsteps*skew(om));
    end
    
end