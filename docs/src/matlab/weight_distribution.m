% simple matlab file that can be used to produces the
% sensor weight distribution figure for the Webots Reference Guide
% May 6, 2005, yvan.bourquin@epfl.ch

a=pi/2;  % aperture
g=0.5;   % gaussianWidth
n=10;    % number of rays

t=[-a/2:a/(n-1):a/2];
v=exp(-(t/(a*g)).^2);
w=v/sum(v);

bar(w);
axis([0 n+1 0 max(w)*1.2]);

set(gca,'FontSize',14);
set(gca,'FontWeight','bold');

xlabel('i');
ylabel('wi');
