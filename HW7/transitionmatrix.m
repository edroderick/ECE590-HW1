symbols

t = 90;
al = 0;
a = 0;
d = 0;

t2 = 0;
al2 = -90;
a2 = 0;
d2 = sym("l1");



T1 = [cosd(t), -1*sind(t)*cosd(al), sind(t)*sind(al), a*cosd(t); sind(t), cosd(t)*cosd(al), -1*cosd(t)*sind(al), a*sind(t); 0, sind(al), cosd(al), d; 0,0,0,1]

T2 = [cosd(t2), -1*sind(t2)*cosd(al2), sind(t2)*sind(al2), a2*cosd(t2); sind(t2), cosd(t2)*cosd(al2), -1*cosd(t2)*sind(al2), a2*sind(t2); 0, sind(al2), cosd(al2), d2; 0,0,0,1]

T3 = T1*T2
