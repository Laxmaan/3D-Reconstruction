clear
clc
%%
%   READ POINTS FROM POINTS3D.TXT
%%
points3D = readmatrix('points3D.txt', 'delimiter', ' ');
pts_xyz = points3D(:,2:4);

rotate3d on
axis equal

%%
%   RUNNING RANSAC ON THE READ POINTS AND GET INLIERS
%%
[B, P, inliers1] = ransac1(pts_xyz', 3, 0.5);
%[B, P, inliers1] = ransacfitplane(pts_xyz', 0.5);

outlier1 = setdiff(1:7842,inliers1);


inliers = pts_xyz(inliers1,:);

%%
%   OBTAIN BEST FIT PLANE THROUGH INLIERS TO DRAW FIGURES ON
%%

mean_xyz = mean(inliers);
new_pts_xyz = inliers - mean_xyz;

plot3(inliers(:,1), inliers(:,2), inliers(:,3),'r.','LineWidth',0.5);
set(gca, 'CameraUpVector', [0 -1 0.1]);
hold on
plot3(pts_xyz(outlier1',1), pts_xyz(outlier1',2), pts_xyz(outlier1',3),'b*','LineWidth',0.5);
hold on
drawnow
rotate3d on
axis equal


mkdir outs %outputdir
saveas(gcf,"outs/pointcloud.png");
%%
%   OBTAIN TRANSFORM MATRIX TO CONVERT FROM LOCAL <---> SCENE SYSTEMS
%%
params = fit_plane_for_given_pts(inliers');
a = params(1);
b = params(2);
c = params(3);

v = [a; b; c];


v = v/norm(v);
k = [0; 0; 1];

theta = acos(v'*k);  %angle between nomal to global plane (Z axis) and 
                     %plane normal
a = cross(v,k);
a = a/norm(a);
A = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
%Rot = axang2rotm( [cross(v,k)' theta]); % rotate by theta by axis perpendicular
                                        % to both normals

Rot = eye(3) + sin(theta)* A + (1 - cos(theta))* (A*A);
scene2local = trvec2tform(-mean_xyz*Rot') * rotm2tform(Rot);
local2scene = pinv(scene2local);

%% plot dominant plane

verts_plane = [ -4 -4 0; -4 4 0; 4 4 0; 4 -4 0];
verts_plane = hom2cart(cart2hom(verts_plane) * local2scene');
faces_plane = [1 2 3 4];
patch('Vertices',verts_plane,'Faces',faces_plane,'FaceVertexCData',[0; 6; 4; 3],'FaceColor','interp')




%%
%   DEFINE BOTH OBJECTS
%%
A = 1.5;
t_object = [0 2*A 0];



verts1 = cart2hom(readmatrix('vertices.txt', 'delimiter', ' ')); %read dolphin

dolphin_center = mean(verts1,1);
verts1 = verts1 * trvec2tform(-hom2cart(dolphin_center))' * rotm2tform(5*eye(3));
local_transform = get_transform_mat(120,0,0,-1.5,4,2.25,"deg")*...
    get_transform_mat(0,0,0,0,0,3,"deg"); % set dolphin on plane

faces1 = readmatrix('faces.txt', 'delimiter', ' '); % read dolphin faces
verts1 = (local2scene * local_transform * verts1')';
verts1 = hom2cart(verts1);




%%% align the cube and draw
[verts2,faces2] = draw_cube(A);
 verts2 = (local2scene * get_transform_mat(0,0,95,1,2,0,"deg") * verts2')';
 verts2 = hom2cart(verts2);

 
 
%% add the cube to scene
 [verts,faces,cdata ] = combineScene(verts1,faces1,verts2,faces2);
patch('Vertices',verts,'Faces',faces,'FaceVertexCData',cdata,'FaceColor','flat')

hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 0 3]) * local2scene'));
hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 3 0]) * local2scene'));
hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([3 0 0]) * local2scene'));
hold off
 %break here
 

saveas(gcf,"outs/pointcloud_objects.png");
 
%%
%   READ CAMERA PARAMS DEFINE R_T and K
%%
cam_params = readmatrix('cameras.txt', 'delimiter', ' ','NumHeaderLines',3);
width = cam_params(3) ;
height = cam_params(4) ;
focal_length = cam_params(5) ;
delta_x = cam_params(6) ;
delta_y = cam_params(7) ;
skew = cam_params(8);

phi_x = focal_length; % / width;
phi_y = focal_length; % / height;

K = [phi_x, skew, delta_x; 0, phi_y, delta_y; 0, 0, 1];

camera_mat = readmatrix('images.txt', 'delimiter', ' ');
%1 SIMPLE_RADIAL 4000 3000 3036.32 2000 1500 0.0249925

new_camera_mat = zeros(27,37338);
for i = 1:27
new_camera_mat(i,:) = camera_mat(2*i - 1,:);
end

Q_matrix = new_camera_mat(:,2:5);
T_matrix = new_camera_mat(:,6:8);

R_matrix = quat2rotm(Q_matrix);

R_T_matrix = zeros(3,4,27);
for i = 1:27
R_T_matrix(:,:,i) = [R_matrix(:,:,i),T_matrix(i,:)'];
end



%%
%   DRAW THE OBJECTS ON THE IMAGES
%%

list_of_files = dir('imgs\*.jpg');
img = [];




% Mark the set_2d points on "img"
for i=1:27
    
    
    %cube 1
    set_2d1 =K * R_T_matrix(:,:,i)* cart2hom(verts)';
    pimage2d1 = hom2cart(set_2d1');
    depths1 = get_Z_buffer(cart2hom(verts)',set_2d1,K,R_T_matrix(:,:,i));
    maxval = max(depths1);
    depths1 = abs(depths1 - maxval);  
    depth_2d1 = [pimage2d1 depths1'];

    
    
    
    figure
    imshow(strcat('imgs\',list_of_files(i).name));
    hold on; 
    
   
    
    patch('Vertices',depth_2d1,'Faces',faces,...
      'FaceVertexCData',cdata,'FaceColor','flat','EdgeAlpha',0);

   colormap('jet');
   
   name = sprintf("outs/op_%d.png",i);
   saveas(gcf,name);
end
