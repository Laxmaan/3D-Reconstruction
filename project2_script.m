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

plot3(inliers(:,1), inliers(:,2), inliers(:,3),'ro','LineWidth',2);
hold on
plot3(pts_xyz(outlier1',1), pts_xyz(outlier1',2), pts_xyz(outlier1',3),'b*','LineWidth',2);
hold on
drawnow
rotate3d on
axis equal

params = fit_plane_for_given_pts(inliers');

%%
%   OBTAIN TRANSFORM MATRIX TO CONVERT FROM LOCAL <---> SCENE SYSTEMS
%%
a = params(1);
b = params(2);
c = params(3);

v = [a; b; c];


v = v/norm(v);
k = [0; 0; 1];

theta = acos(v'*k);  %angle between nomal to global plane (Z axis) and 
                     %plane normal

Rot = axang2rotm( [cross(v,k)' theta]); % rotate by theta by axis perpendicular
                                        % to both normals


scene2local = trvec2tform(-mean_xyz*Rot') * rotm2tform(Rot);
local2scene = pinv(scene2local);

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
cdata1 = randi([1 size(faces1,1)],size(faces1,1),1); % get colors
patch('Vertices',verts1,'Faces',faces1,'FaceVertexCData',cdata1,'FaceColor','flat');


%%% align the cube and draw
[verts2,faces2] = draw_cube(A);
 verts2 = (local2scene * get_transform_mat(0,0,95,1,2,0,"deg") * verts2')';
 verts2 = hom2cart(verts2);
 cdata2 = randi([1 size(faces1,1)],size(faces2,1),1);
 patch('Vertices',verts2,'Faces',faces2,'FaceVertexCData',cdata2,'FaceColor','flat');


hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 0 3]) * local2scene'));
hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([0 3 0]) * local2scene'));
hold on
vectarrow(hom2cart(cart2hom([0 0 0]) * local2scene'),hom2cart(cart2hom([3 0 0]) * local2scene'));
hold off
 %break here
 
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


mkdir outs %outputdir

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
      'FaceVertexCData',cdata1,'FaceColor','flat','EdgeAlpha',0);
   patch('Vertices',depth_2d2,'Faces',faces2,...
       'FaceVertexCData',cdata2,'FaceColor','flat','EdgeAlpha',0);
   colormap('jet');
   
   name = sprintf("outs/op_%d.png",i);
   saveas(gcf,name);
end
