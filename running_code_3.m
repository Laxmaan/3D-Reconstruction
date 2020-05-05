clear
clc
points3D = readmatrix('points3D.txt', 'delimiter', ' ');
pts_xyz = points3D(:,2:4);

rotate3d on
axis equal

[B, P, inliers1] = ransac1(pts_xyz', 3, 0.5);
%[B, P, inliers1] = ransacfitplane(pts_xyz', 0.5);

outlier1 = setdiff(1:7842,inliers1);


table_top = pts_xyz(inliers1,:);


mean_xyz = mean(table_top);
new_pts_xyz = table_top - mean_xyz;

plot3(table_top(:,1), table_top(:,2), table_top(:,3),'ro','LineWidth',2);
hold on
plot3(pts_xyz(outlier1',1), pts_xyz(outlier1',2), pts_xyz(outlier1',3),'b*','LineWidth',2);
hold on
drawnow
rotate3d on
axis equal

params = fit_plane_for_given_pts(table_top');

a = params(1);
b = params(2);
c = params(3);

v = [a; b; c];


v = v/norm(v);
k = [0; 0; 1];

theta = acos(v'*k);

Rot = axang2rotm( [cross(v,k)' theta]);


scene2local = trvec2tform(-mean_xyz*Rot') * rotm2tform(Rot);
local2scene = pinv(scene2local);
A = 1;
t_object = [0 2*A 0];
local_transform = get_transform_mat(0,0,5,-A,2*A,0,"deg");
[verts1,faces1] = draw_cube(A);
[verts2,faces2] = draw_cube(A);


verts1 = (local2scene * local_transform * verts1')';
verts1 = hom2cart(verts1);
patch('Vertices',verts1,'Faces',faces1,'FaceVertexCData',(1:6)','FaceColor','flat');

verts2 = (local2scene * get_transform_mat(0,0,95,A,2*A,0,"deg") * verts2')';
verts2 = hom2cart(verts2);
patch('Vertices',verts2,'Faces',faces2,'FaceVertexCData',(1:6)','FaceColor','flat');


hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 0 3]) * local2scene'));
hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 3 0]) * local2scene'));
hold on
 vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([3 0 0]) * local2scene'));
 hold off

%1 SIMPLE_RADIAL 4000 3000 3035.22 2000 1500 0.01192
width = 4000 ;
height = 3000 ;
focal_length = 3036.32 ;
delta_x = 2000 ;
delta_y = 1500 ;
skew = 0.0249925;

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





list_of_files = dir('imgs\*.jpg');
img = [];


%img = reshape(img,[2268, 4032, 37]);
%img = reshape(img,[3000, 4000, 27]);

% Mark the set_2d points on "img"
for i=1:27
    
    
    %cube 1
    set_2d1 =K * R_T_matrix(:,:,i)* cart2hom(verts1)';
    pimage2d1 = hom2cart(set_2d1');
    depths1 = get_Z_buffer(cart2hom(verts1)',set_2d1,K,R_T_matrix(:,:,i));
    
    %cube 2
    set_2d2 =K * R_T_matrix(:,:,i)* cart2hom(verts2)';
    pimage2d2 = hom2cart(set_2d2');
    depths2 = get_Z_buffer(cart2hom(verts2)',set_2d2,K,R_T_matrix(:,:,i));
    
    %combine depths
    maxval = max([depths1 depths2]);
    depths1 = abs(depths1 - maxval);
    depth_2d1 = [pimage2d1 depths1'];
    depths2 = abs(depths2 - maxval);
    depth_2d2 = [pimage2d2 depths2'];
    
    
    
    
    figure
    imshow(strcat('imgs\',list_of_files(i).name));
    hold on; 
    
    
    patch('Vertices',depth_2d1,'Faces',faces1,...
      'FaceVertexCData',(1:6)','FaceColor','flat','EdgeAlpha',0);
   patch('Vertices',depth_2d2,'Faces',faces2,...
       'FaceVertexCData',(1:6)','FaceColor','flat','EdgeAlpha',0);
end
