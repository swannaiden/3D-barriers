cfg = coder.config('lib');
cfg.GenCodeOnly = true;
cfg.TargetLang = 'C++';
cfg.FilePartitionMethod = 'SingleFile';
cfg.RuntimeChecks = 1;
cfg.BuildConfiguration = 'Debug';

% codegen -config cfg cont_dynamics -args {zeros(1,1,'double'),zeros(13,1,'double'),zeros(4,1,'double')}
codegen -config cfg barrier -args {zeros(13,1,'double'),zeros(4,1,'double')}
% codegen -config cfg zBodyInWorld -args {zeros(4,1,'double')}
% codegen -config cfg affine_dynamics -args {zeros(13,1,'double')}
% codegen -config cfg JfFun -args {zeros(13,1,'double')}
% codegen -config cfg JgFun -args {zeros(13,1,'double')}
% codegen -config cfg Dfcl -args {zeros(13,1,'double'),zeros(4,1,'double')}
% x = sym('x',[13,1],'real');
% hh = sym('hh',[1,1],'real');
% y0 = sym('y0',[1,1],'real');
% u = backup_controller_sim(x,hh,y0);
% Du = jacobian(u,x);
% matlabFunction(Du,'Vars',{x,hh,y0},'File','DuFun');

% computeGradientDynamics(@affine_dynamics,[13,4])

function computeGradientDynamics(funHandle,dim)
x = sym('x', [dim(1),1],'real');
u = sym('u', [dim(2),1],'real');

[f,g] = funHandle(x);
Jf = jacobian(f,x);
Jg = sym('Jgs', [dim(1),dim(2),dim(1)],'real');
Dfcl = Jf;

for i = 1:dim(2)
    Jg(:,i,:) = jacobian(g(:,i),x);
    Dfcl = Dfcl + jacobian(g(:,i),x)*u(i);
end

for i = 1:dim(1)
    for j = 1:dim(1)
        Dfcl2(i+(j-1)*dim(1)) = Jf(i,j);
        for k = 1:dim(2)
            Dfcl2(i+(j-1)*dim(1)) = Dfcl2(i+(j-1)*dim(1)) + Jg(i,k,j)*u(k);
        end
    end
end
Dfcl2 = reshape(Dfcl2,dim(1),dim(1));

matlabFunction(Jf,'Vars',{x},'File','JfFun');
matlabFunction(Jg,'Vars',{x},'File','JgFun');
matlabFunction(Dfcl,'Vars',{x,u},'File','Dfcl');
end