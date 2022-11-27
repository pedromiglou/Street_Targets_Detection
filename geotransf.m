function T = geotransf(x,y,z,xalpha,yalpha,zalpha)
    T = [ 1 0      0       0
      0 cos(xalpha) -sin(xalpha) 0
      0 sin(xalpha) cos(xalpha)  0
      0 0      0       1
    ] * [ cos(yalpha)  0 sin(yalpha)  0
      0       1 0       0
      -sin(yalpha) 0 cos(yalpha)  0
      0       0 0       1
    ] * [ cos(zalpha)  -sin(zalpha) 0  0
      sin(zalpha)  cos(zalpha)  0  0
      0       0       1  0
      0       0       0  1
    ];

    T(1,4) = x;
    T(2,4) = y;
    T(3,4) = z;
end

