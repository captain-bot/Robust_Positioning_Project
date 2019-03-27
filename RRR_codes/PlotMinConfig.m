function [] = PlotMinConfig(p1,p2,p3,xd,yd)
    x = [0 p1(1) p2(1) p3(1)];
    y = [0 p1(2) p2(2) p3(2)];
    plot(p1(1), p1(2),'o', p2(1), p2(2), 'o', p3(1), p3(2))
    plot(x, y, 'LineWidth', 2.0)
    title('Minimum Error Bound Configuration')
    text(0,0,' base')
    text(0.91,0.94,'radius l_2 + l_3')
    text(-0.5,-0.2,'radius l_1')
    text(0.2,0.03,' l_1')
    text(0.3,0.46,' l_2')
    text(0.46,0.65,' l_3')
    text(xd,yd,' desired point')
    
    % print the figure in eps format
    print -depsc rrrmin.eps
end