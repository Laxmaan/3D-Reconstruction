function [vert_final,faces_final,cdata] = combineScene(vertices1,faces1,vertices2, faces2)
%Adds object denoted by vertices2, faces2 to the scene with
%vertices1,faces1. Vertices are all in 3D cartesian form

sizes = [size(faces1,2); size(faces2,2)];
face_size = max(sizes);

%extend faces1
n_to_extend = face_size - sizes(1);
if n_to_extend > 0
    faces1 = [faces1 faces1(:,1:n_to_extend)];
end

%extend faces2
n_to_extend = face_size - sizes(2);
if n_to_extend > 0
    faces2 = [faces2 faces2(:,1:n_to_extend)];
end

n_vertices_1 = size(vertices1,1);

faces2 = faces2 + n_vertices_1;
vert_final = [vertices1; vertices2];
faces_final = [faces1 ; faces2];
cdata = randi([1 size(faces_final,1)],size(faces_final,1),1);
end

