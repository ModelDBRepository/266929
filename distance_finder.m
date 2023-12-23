function [midx,midy,tangx,tangy,eid]=distance_finder(cf,dt) 
cs = cscvn(cf(:,1:5:end));
points = fnplt(cs,2);
mx = mean(points(1,:));
my = mean(points(2,:));

dx = (points(1,:)-mx).^2;
dy = (points(2,:)-my).^2;
ct = 90;
dd = 20;

%% compute the horizontal cross sectional area 
[pm,pl] = findpeaks(-dx,'minpeakheight',prctile(-dx,ct),'minpeakdistance',dd);
px = points(1,pl);
py = points(2,pl);
midx = [px;py];
mx = mean(points(1,:));
my = mean(points(2,:));
%% compute min distance via slope-tangent 
i1 = min(pl(1)+dt,length(points(2,:)));
i2 = max(pl(1)-dt,1);

dydx = (points(2,i1)-points(2,i2))/(points(1,i1)-points(1,i2));
slope = -1/dydx; 
xhat = -400:0.1:400; 
yhat = slope*(xhat) + points(2,pl(1));
xhat = points(1,pl(1)) + xhat;
for i = 1:length(points)
min_dis(i) = min((points(1,i) - xhat).^2 + (points(2,i)-yhat).^2);    
end
[pm,pl] = findpeaks(-min_dis,'minpeakheight',prctile(-min_dis,ct),'minpeakdistance',dd);
px = points(1,pl);
py = points(2,pl); 
tangx = [px;py];


%% compute the vertical cross sectional area 
[pm,pl] = findpeaks(-dy,'minpeakheight',prctile(-dy,ct),'minpeakdistance',dd);
px = points(1,pl);
py = points(2,pl);
midy = [px;py];

%% compute the 
i1 = min(pl(1)+dt,length(points(2,:)));
i2 = max(pl(1)-dt,1);
dydx = (points(2,i1)-points(2,i2))/(points(1,i1)-points(1,i2));
slope = -1/dydx; 
xhat = -400:0.1:400; 
yhat = slope*(xhat) + points(2,pl(1));
xhat = points(1,pl(1)) + xhat;
for i = 1:length(points)
min_dis(i) = min((points(1,i) - xhat).^2 + (points(2,i)-yhat).^2);    
end
[pm,pl] = findpeaks(-min_dis,'minpeakheight',prctile(-min_dis,ct),'minpeakdistance',dd);
px = points(1,pl);
py = points(2,pl); 
tangy = [px;py];




eid(4) = size(tangy,2);
eid(3) = size(tangx,2);
eid(2) = size(midy,2);
eid(1) = size(midx,2);



end
%%


