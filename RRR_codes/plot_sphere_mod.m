function [x, y, z] = plot_sphere_mod(c, r, varargin)

    opt.color = [0,0,0];
    opt.alpha = 0.5;
    opt.mesh = [0.1,0.2,0.4];

    [opt,args] = tb_optparse(opt, varargin);
    
    % backward compatibility with RVC
    if length(args) > 0
        opt.color = args{1};
    end
    if length(args) > 1
        opt.alpha = args{2};
    end
    
    daspect([1 1 1])
    hold_on = ishold 
    hold on
    [xs,ys,zs] = sphere(40);

    if isvec(c,3)
        c = c(:);
    end
    if size(r) == 1
        r = r * ones(numcols(c),1);
    end

    if nargin < 4
        alpha = 1
    end

    % transform the sphere
    for i=1:numcols(c)
        x = r(i)*xs + c(1,i);
        y = r(i)*ys + c(2,i);
        z = r(i)*zs + c(3,i);
                
        % the following displays a nice smooth sphere with glint!
        %h = surf(x, y, z, 'edgecolor','none')
        %h = surf(x,y,z, 'FaceColor', opt.color, 'EdgeColor', opt.mesh, 'FaceAlpha', opt.alpha);
        %h = surf(x,y,z, 'FaceColor', opt.color, 'FaceAlpha', opt.alpha)
        % camera patches disappear when shading interp is on
        %h = surfl(x,y,z)
        h = mesh(x, y, z)
       
    end
    %lighting gouraud
    %light
    if ~hold_on
        hold off
    end