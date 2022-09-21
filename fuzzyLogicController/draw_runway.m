function [V,F,facecolors] = define_runway

  fuse_l1    = 7;
  fuse_l2    = 4;
  fuse_l3    = 15;
  fuse_w     = 2;
  fuse_h     =fuse_w; 
  wing_l     = 6;
  wing_w     = 20;
  tail_l     = 3;
  tail_h     = 3;
  tailwing_w = 10;
  tailwing_l = 3;
  
  scale = 5;
  
 
% Define the vertices (physical location of vertices
V = [...
    fuse_l1, 0, 0;...   % pt 1
    fuse_l2, fuse_w/2, -fuse_h/2;... % pt 2
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l2, -fuse_w/2, fuse_h/2;...  % pt 4
    fuse_l2, fuse_w/2, fuse_h/2;...  % pt 5
    -fuse_l3, 0, 0;...  % pt 6
    0, wing_w/2, 0;...  % pt 7
    -wing_l, wing_w/2, 0;...  % pt 8
    -wing_l, -wing_w/2, 0;...  % pt 9
    0, -wing_w/2, 0;...  % pt 10
    -(fuse_l3-tailwing_l), tailwing_w/2, 0;...  % pt 11
    -fuse_l3, tailwing_w/2, 0;...  % pt 12
    -fuse_l3, -tailwing_w/2, 0;...  % pt 13
    -(fuse_l3-tailwing_l), -tailwing_w/2, 0;...  % pt 14
    -(fuse_l3-tailwing_l), 0, 0;...  % pt 15
    -fuse_l3, 0, -tail_h;...  % pt 16
    ]';

V=V*scale;

  F = [...
        1, 2, 3, 1;...  % head top
        1, 3, 4, 1;...  % head right
        1, 2, 5, 1;...  % head left
        1, 5, 4, 1;...  % head bottom
        2, 3, 6, 2;...  %body top
        3, 4, 6, 3;...  %body left
        2, 5, 6, 2;...  %body right
        4, 5, 6, 4;...  %body bottom
        6, 15, 16, 6;... %tail
        7, 8, 9, 10;...  %wing diagonal 1
        11, 12, 13, 14;...  %tail wing dia 1      
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    myyellow;...    % head top
    myyellow;...    % head right
    myyellow;...    % head left 
    myyellow;...    % head bottom 
    myblue;...     % body top 
    myblue;...     % body left 
    myblue;...     % body right
    myred;...     % body bottom
    myblue;...     % tail
    mycyan;...  %wing diagonal 1
    mycyan;...  %tail wing dia 1
    ];
end