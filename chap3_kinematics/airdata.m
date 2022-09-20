function out=airdata(uu)
%
% Fake air data - this will be replaced with real air data in Chapter 4
%
% modified 12/11/2009 - RB

    % Process inputs to function
    u = uu(4); % body velocity along x-axis (m/s)
    v = uu(5); % body velocity along y-axis (m/s)
    w = uu(6); % body velocity along z-axis (m/s)
    
    % Define air data
    Va    = sqrt(u^2+v^2+w^2);
    alpha = atan2(w,u);
    beta  = asin(v);
    wn    = 0;
    we    = 0;
    wd    = 0;
    
    out = [Va; alpha; beta; wn; we; wd];
    