%%  Quick tracker algorithm.  Note that each ROI generates 6 total tracked quantities in the excel file 
% 1) Cross sectional area 
% 2) normalized cross sectional area, divided by the initial template 
% 3) Horizontal Distance of line through mean mask coordinate 
% 4) Vertical Distance of line through mean mask coordinate
% 5) Tangent line distance through horizontal mean-mask coordinate
% 6) Tangent line distance through vertical mean-mask coordinate
clear all
close all
clc


%% load tiff stack 
[filename_cell,pathname_cell] = uigetfile('*.*',  'All Files (*.*)','MultiSelect','on');
multi_image = iscell(filename_cell); 

% used to initialize if we load mulitple images at once. 
if multi_image == 0
num_images = 1; 
filename_cell_ = cell(1);
filename_cell_{1} = filename_cell; 
filename_cell = filename_cell_; 
end



%% Run through all images to find ROIs for automated alignment 
num_images = size(filename_cell,2)';
for nim = 1:num_images 
filename = filename_cell{nim};
pathname = pathname_cell;
im_info = imfinfo(sprintf('%s/%s',pathname,filename)); % return tiff structure, one element per image
nframes_ = size(im_info,1);
nframes(nim) = nframes_;
frame_val = input('Select Frame Index   ');

figure(1)
Mim = 0;
        for nf = 1:length(frame_val)
         im0 = imread(sprintf('%s/%s',pathname,filename), frame_val(nf)) ; % read in first image
         im0 = im2double(im0);         
Mim = Mim + im0/length(frame_val);
        end
im = Mim;
imagesc(im)
set(gca,'ydir','normal')
commandwindow;
apply_filter_ = input('Apply Filtering to Image (1 or 0)? ');


if apply_filter_ == 1 
figure(1)
cla
im = wiener2(im,[10,10]);
imagesc(im,[0,1])
set(gca,'ydir','normal')
end 
commandwindow;




load_roi = input('Load ROIs (1 or 0)?  ');


%% generate new rois 
if load_roi==0 

num_roi_ = input('Select Number of Regions of Interest  ');
num_roi(nim) = num_roi_;
appfilter(nim) = apply_filter_;

%% 
for j = 1:num_roi(nim)

% process ROIss     
ii = 1;
ntot = min(size(im,1),size(im,2));
im = im(1:ntot,1:ntot);
figure(1) 
subplot(1,2,1)
cla
imagesc(im), hold on 
set(gca,'ydir','normal')
rect = getrect;
xmin = rect(1,1); ymin = rect(1,2); 
width = rect(1,3); 
height = rect(1,4);
title('Select a Region to Track')
commandwindow;
deff{nim,j} = input('Is ROI deformable (1 or 0)?  ');

subplot(1,2,2)
cla
nx = round(xmin):1:round(xmin+width);
ny = round(ymin):1:round(ymin+height);
im_template = 0*im;
im_template = im(ny,nx);

imagesc(im_template), hold on 
set(gca,'ydir','normal')
title('Click to select bounding region, Press Ctrl + C to stop')



tdot = 0; 
p = []; 
while tdot == 0 
 [x,y] = ginput(1); 
 p = [p;[x,y]];
 plot(x,y,'r.','MarkerSize',10)
tdot = waitforbuttonpress;
end


p = round(p);
p = [p;p(1,:)];
[MX,MY] = meshgrid(1:size(im_template,2),1:size(im_template,1));
[mask,mask_contour] = inpolygon(MX,MY,p(:,1),p(:,2));
close(figure(1))
figure(1)
title('Select Box Area')
imagesc(mask)
set(gca,'ydir','normal')
rect = getrect;
xmin = rect(1,1); ymin = rect(1,2); 
width = rect(1,3); 
height = rect(1,4);
nax = round(xmin):1:round(xmin+width);
nay = round(ymin):1:round(ymin+height);
% nax(nax<min(nx))=[];
% nax(nax>max(nx))=[]; 
% nay(nay<min(ny))=[];
% nay(nay>max(ny))=[];
% 

area_mask = 0*im_template;
area_mask(nay,nax) = 1;

store_mask{nim,j} = mask;
store_area_mask{nim,j} = area_mask;
store_nx{nim,j} = nx;
store_ny{nim,j} = ny;
store_template{nim,j} = im_template;
close all
end


save_roi = input('Save ROIs (1 or 0)?  ');

if save_roi==1 
temp_mask{1,:} = store_mask{nim,:};
temp_store_area_mask{1,:} = store_area_mask{nim,:};
temp_store_nx{1,:} = store_nx{nim,:}; 
temp_store_ny{1,:} = store_ny{nim,:}; 
temp_store_template{1,:} = store_template{nim,:};
temp_deff{1,:} = deff{nim,:};
uisave({'temp_mask','temp_store_area_mask','temp_store_nx','temp_store_ny','temp_store_template','num_roi_','ntot','apply_filter_','temp_deff'});
end

end


% load old ROIs 
if load_roi == 1 
uiload 
num_roi(nim) = num_roi_;
appfilter(nim) = apply_filter_;
for j = 1:num_roi_
store_mask{nim,j} = temp_mask{1,j};
store_area_mask{nim,j} = temp_store_area_mask{1,j};
store_nx{nim,j} = temp_store_nx{1,j};
store_ny{nim,j} = temp_store_ny{1,j};
store_template{nim,j} = temp_store_template{1,j};     
deff{nim,:} = temp_deff{1,:};
end
end
    
    
    
end 












%%  Run over the total number of images
for nim = 1:num_images
filename = filename_cell{nim};
pathname = pathname_cell;
time_str = datestr(now,30);
v = VideoWriter(sprintf('video_%s_%s',filename(1:end-4),time_str),'Motion JPEG AVI');
v.Quality = 95;
v.FrameRate = 10;
open(v);
axis tight manual
tic
SA = zeros(nframes(nim),num_roi(nim));
SAN = SA; MX = NaN + SA; MY = NaN + SA; DTX = NaN + SA; DTY = NaN+SA; 
close all
for j = 1:nframes(nim)   
    
    
    
   %% load the current frame or a moving average of frames. 
  
   im_current = imread(sprintf('%s/%s',pathname,filename), j) ; % read in first image
   im_current = im2double(im_current);
   im_current = im_current(1:ntot,1:ntot);  
   im_comp = im_current;   
   
   if appfilter(nim)==1
   im_current = wiener2(im_current,[10,10]);    
   end
   
   f2 = figure(2);
   drawnow 
   
   
   
   for nroi = 1:num_roi(nim)
   mask = store_mask{nim,nroi};
   area_mask = store_area_mask{nim,nroi};
   nx = store_nx{nim,nroi};
   ny = store_ny{nim,nroi} ;
   im_template= store_template{nim,nroi};
   im_fixed = im_current(ny,nx); 
   im_comp_fixed = im_comp(ny,nx);

   
   
   if deff{nim,nroi}==1 
   [D,tformimg] = imregdemons(im_template,im_fixed,'DisplayWaitBar',false);
   [mask_warp] = double(imwarp(mask,D));
   else 
   tformimg = im_template;
   mask_warp = double(mask);     
   end
   
   %% plot the raw frame 
   subplot(num_roi(nim),4,(nroi-1)*4+1)
   imagesc(im_comp_fixed)
   daspect([1,1,1])
   set(gca,'ydir','normal')
   if nroi == 1 
   title('Raw Current Frame')
   end
   %% plot the transformed template and compute mask transform
   subplot(num_roi(nim),4,(nroi-1)*4+3)
   imagesc(tformimg)
     daspect([1,1,1])
   set(gca,'ydir','normal')
   if nroi == 1 
    title('Warped Template')   
   end
   colormap('hot')
   
   %% plot the transformed mask 
   subplot(num_roi(nim),4,(nroi-1)*4+4)
   imagesc(mask_warp + mask_warp.*area_mask,[0,2])
     daspect([1,1,1])
   set(gca,'ydir','normal')
      if nroi == 1 
    title('Warped Mask')   
   end
   
   %% plot the contour and the original image
   subplot(num_roi(nim),4,(nroi-1)*4+2)
   cf = contourc(mask_warp,[1,1]);

   cf(:,cf(1,:)>prctile(cf(1,:),99))=[];
   cf(:,cf(2,:)>prctile(cf(2,:),99))=[];
   cf(:,cf(1,:)<prctile(cf(1,:),1))=[];
   cf(:,cf(2,:)<prctile(cf(2,:),1))=[];
      
   cf = [cf,cf(:,1)];
   dt = 10;
   [midx,midy,tangx,tangy,eid]=distance_finder(cf,dt);

  
   imagesc(im_fixed), hold on 
   plot(cf(1,:),cf(2,:),'b','LineWidth',2)
   plot(midx(1,:),midx(2,:),'m','LineWidth',2)
   plot(midy(1,:),midy(2,:),'m','LineWidth',2)
   plot(tangx(1,:),tangx(2,:),'m','LineWidth',2)
   plot(tangy(1,:),tangy(2,:),'m','LineWidth',2)
   daspect([1,1,1])
   set(gca,'ydir','normal')
   SA(j,nroi) = sum(sum(mask_warp.*area_mask));
   INT(j,nroi) = sum(sum(mask_warp.*area_mask.*im_comp_fixed));
   
   
   if eid(1)==2
   MX(j,nroi) = sqrt((midx(1,1)-midx(1,2)).^2 + (midx(2,1) - midx(2,2)).^2);
   end
   if eid(2)==2
   MY(j,nroi) = sqrt((midy(1,1)-midy(1,2)).^2 + (midy(2,1) - midy(2,2)).^2);
   end
   if eid(3) == 2 
   DTX(j,nroi) = sqrt((tangx(1,1)-tangx(1,2)).^2 + (tangx(2,1) - tangx(2,2)).^2);
   end
   if eid(4) == 2 
   DTY(j,nroi) = sqrt((tangy(1,1)-tangy(1,2)).^2 + (tangy(2,1) - tangy(2,2)).^2);
   end
   
   if nroi == 1 
    title('Filtered Current Frame')   
   end
   hold off

   end
      sgtitle(sprintf('Frame Number %d',j))
   set(f2,'position',[0,0,900,300*num_roi(nim)])
   writeVideo(v,getframe(gcf))
   
end



close(v)
toc
%% 
%time = nframes/v.FrameRate; 
% 
% figure(22)
%% normalize area by template area;
 for d = 1:num_roi(nim) 
 null_area = sum(sum(store_mask{nim,d}.*store_area_mask{nim,d}));
SAN(:,d) = SA(:,d)/null_area;
 end
close all
data_table = array2table([INT,SA,SAN,MX,MY,DTX,DTY]);
var_names_temp = {'Intensity','Cross_Sectional_Area','Cross_Sectional_Area_Normalized','Horizontal_Mean_Distance','Vertical_Mean_Distance','Horizontal_Tangent_Point_Distance','Vertical_Tangent_Point_Distance'};
for i = 1:7*nroi
    roi = mod(i-1,nroi)+1;
    q = ceil(i/nroi);
    data_table.Properties.VariableNames{sprintf('Var%d',i)}=sprintf('%s_ROI_%d',var_names_temp{q},roi);

end

filename = sprintf('areas_%s_%s.xlsx',filename(1:end-4),time_str);
writetable(data_table,filename)

save(sprintf('processed_%s_%s.mat',filename(1:end-4),time_str),'-v7.3')
end



