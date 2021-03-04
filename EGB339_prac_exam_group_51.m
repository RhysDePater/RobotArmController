%% Solution for EGB339 prac for 2020




function [init_xy, dest_xy] = EGB339_prac_exam_group_51(dobot_sim, init_positions, dest_positions, use_vision)
    
    if use_vision == true
        
    % do computer vision on initial_positions and destination_positions arguments which are images
    imPos=init_positions;
    imDes=dest_positions;

    %for initPositions
    pos = structShape(imPos);
    
    %for desPositions
    des=structShape(imDes);
    
    sync = false;
    %dobot_sim = coppeliaRobot('Dobot');
    dobot_sim.startSim(sync);
    
    target = [0 0 -45 45 0];
    
    dobot_sim.setJointPositions(target);
    
    while(compareAngles(dobot_sim.getJointPositions(), target))
    end
    image = dobot_sim.getImage();
    image = image(185:end,:,:);
    %figure;imshow(image);
    
    homog = cameraHomography(image);
    sheetPos = destShapeClassifier(image,homog);
    
    init_xy = compareShapes(sheetPos, pos);
    dest_xy = compareShapes(sheetPos, des);
    else

        %% Execute Pick and Place

        % This section iterates through each start and end coordinates
        % to pick up each object and place at the desired location
        sync = false;
        %dobot_sim = coppeliaRobot('Dobot');
        dobot_sim.startSim(sync);
        for i=1:3
            moveArm(init_positions(i,1),init_positions(i,2),dobot_sim);
            dobot_sim.setSuctionCup(true);
            
            x = init_positions(i,1)- 80;
            y = init_positions(i,2) - 290;
            
            theta1 = atan2d(y, x);
            
            target = [theta1, 0,-20,20,0];
            dobot_sim.setJointPositions(target);
            while(compareAngles(dobot_sim.getJointPositions(), target))
            end
            
            moveArm(dest_positions(i,1),dest_positions(i,2),dobot_sim);
            dobot_sim.setSuctionCup(false);
            
            target = inverseKinematics(dest_positions(i,1)-80,dest_positions(i,2)-290,60);
            
            dobot_sim.setJointPositions(target);
            while(compareAngles(dobot_sim.getJointPositions(), target))
            end
        end
    end
    

end

function pos_xy = compareShapes(sheetPos, pos)
pos_xy = [0,0; 0,0; 0,0];
    for i=1:length(pos)
        pos1 = [0,0,0];
        pos2 = [0,0];
       for j=1:length(sheetPos)
           if pos(i).shape == sheetPos(j).shape
               if pos(i).Colour == sheetPos(j).Colour
                  if pos1(1,1) == 0
                      pos1 = [sheetPos(j).Centroid(1,1),sheetPos(j).Centroid(1,2),sheetPos(j).Area];
                  else
                      pos2 = [sheetPos(j).Centroid(1,1),sheetPos(j).Centroid(1,2),sheetPos(j).Area];
                  end
               end
           end
       end
       if (pos(i).size == "big")
           if (pos1(1,3)>pos2(1,3))
               pos_xy(i,1:2) = pos1(1,1:2);
           else
               pos_xy(i,1:2) = pos2(1,1:2);
           end
       else
           if (pos1(1,3)<pos2(1,3))
               pos_xy(i,1:2) = pos1(1,1:2);
           else
               pos_xy(i,1:2) = pos2(1,1:2);
           end
       end
    end
end

function moveArm(x,y,dobot_sim)
x = x- 80;
y = y - 290;

theta1 = atan2d(y, x);

target = [theta1, 0,-20,20,0];

dobot_sim.setJointPositions(target);

while(compareAngles(dobot_sim.getJointPositions(), target))
end

target = inverseKinematics(x,y,60);

dobot_sim.setJointPositions(target);
while(compareAngles(dobot_sim.getJointPositions(), target))
end

target = inverseKinematics(x,y,53);

dobot_sim.setJointPositions(target);
while(compareAngles(dobot_sim.getJointPositions(), target))
end
end

function target = inverseKinematics(x,y,z)
    % test coordinates;

% Robot joint lengths
l1 = 138;
l2 = 135;
l3 = 147;
l4 = 60;
l5 = 75;

L = sqrt(y^2 + x^2) - l4;
 % height of cyclinder
H = z + l5;
V = H - l1;
a = sqrt(L^2 + V^2);
an = (l2^2 + l3^2 - a^2)/(2*l2*l3);
A = atan2d(sqrt(1 - an^2), an);
D = (l2^2+a^2-l3^2)/(2*l2*a);
C = atan2d(sqrt(1-D^2), D); %asind(l3*sin(A)/a)
phi = atan2d(V,L);

theta1 = atan2d(y, x);
theta2 = 90 - C - phi;
theta3 = 90 - A;
theta4 = - theta3 - theta2;
theta5 = 90;

target = [theta1 theta2 theta3 theta4 theta5 ];
end

function bool = compareAngles(angle1, angle2)
    bool = false;
    for i=1:5
        if (abs(angle1(i)-angle2(i))<0.05)
            bool = false;
        else
            bool = true;
            break
        end
    end 
end


function homog = cameraHomography(image)
image = double(image)/255;

R = image(:,:,1);
G = image(:,:,2);
B = image(:,:,3);
intensity = R+G+B;

b = B./intensity;
BWB = b>=0.7;
regionBWB = regionprops(BWB, 'Centroid');
centroids = cat(1,regionBWB.Centroid).';
%figure(); imshow(BWB);

realCoords = [345 182.5 20 20 345 182.5 345 182.5 20;560 560 560 290 290 290 20 20 20];
homog = homography(centroids, realCoords);
end

function pos = destShapeClassifier(image,homog)
image = double(image)/255;

R = image(:,:,1);
G = image(:,:,2);
B = image(:,:,3);
intensity = R+G+B;

g = G./intensity;
BWG = g>=0.7;
BWG = imfill(BWG,'holes');
r = R./intensity;
BWR = r>=0.7;
BWR = imfill(BWR,'holes');
%figure(); imshow(BWG);
%figure(); imshow(BWR);
BWGRegion = regionprops(BWG, "Area", "Circularity","Centroid");
BWRRegion = regionprops(BWR, "Area", "Circularity","Centroid");

pos1.shape ="";
pos2.shape ="";
pos3.shape ="";
pos4.shape ="";
pos5.shape ="";
pos6.shape ="";
pos7.shape ="";
pos8.shape ="";
pos9.shape ="";
pos10.shape ="";
pos11.shape ="";
pos12.shape ="";

pos = [pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,pos9,pos10,pos11,pos12];
for i=1:length(BWGRegion)
    if BWGRegion(i).Circularity >=0.9
        
        pos(i).shape = "Circle";
    elseif BWGRegion(i).Circularity <=0.68
        pos(i).shape = "Triangle";
    else
        pos(i).shape = "Square";
    end
    pos(i).Area=BWGRegion(i).Area;
    realpoint = homog*[BWGRegion(i).Centroid(1,1);BWGRegion(i).Centroid(1,2);1];
    realpoint = [realpoint(1,1)/realpoint(3,1);realpoint(2,1)/realpoint(3,1)];
    pos(i).Centroid=[realpoint(1,1),realpoint(2,1)];
    pos(i).Colour="Green";
end
for i=1:length(BWRRegion)
    if BWRRegion(i).Circularity >=0.9
        
        pos(i+6).shape = "Circle";
    elseif BWRRegion(i).Circularity <=0.68
        pos(i+6).shape = "Triangle";
    else
        pos(i+6).shape = "Square";
    end
    pos(i+6).Area=BWRRegion(i).Area;
    realpoint = homog*[BWRRegion(i).Centroid(1,1);BWRRegion(i).Centroid(1,2);1];
    realpoint = [realpoint(1,1)/realpoint(3,1);realpoint(2,1)/realpoint(3,1)];
    pos(i+6).Centroid=[realpoint(1,1),realpoint(2,1)];
    pos(i+6).Colour="Red";
end
end

function [pos] = structShape(imPos)
%for initPositions
colourThreshold = 3;
imageThreshold = 0.7;
imPosBW = rgb2gray(imPos);
imPosBW = ~imbinarize(imPosBW);
imPosBW = imfill(imPosBW,'holes');
imPosBW = medfilt2(imPosBW);
%figure; imshow(imPosBW);

posRegion = regionprops(imPosBW, "Area", "Circularity","Centroid");

pos1.shape ="";
pos2.shape ="";
pos3.shape ="";

pos = [pos1,pos2,pos3];
for i=1:length(posRegion)
    if posRegion(i).Circularity >=0.9
        
        pos(i).shape = "Circle";
    elseif posRegion(i).Circularity <=0.65
        pos(i).shape = "Triangle";
    else
        pos(i).shape = "Square";
    end
    pos(i).Area=posRegion(i).Area;
    pos(i).Centroid=posRegion(i).Centroid;
end

if(length(posRegion) == 3)
    small = 0;
    large = 0;
    smallID = 0;
    midID = 0;
    largeID = 0;
    for i=1:length(posRegion)
        if pos(i).Area<=pos(1).Area &&pos(i).Area<=pos(2).Area &&pos(i).Area<=pos(3).Area
            pos(i).size = "small";
            small = pos(i).Area;
            smallID = i;
        elseif pos(i).Area>=pos(1).Area &&pos(i).Area>=pos(2).Area &&pos(i).Area>=pos(3).Area
            pos(i).size = "big";
            large = pos(i).Area;
            largeID = i;
        else
            midID = i;
        end
    end
    diff = (pos(largeID).Area / pos(midID).Area);
    
    if pos(midID).Area > pos(smallID).Area * diff
        pos(midID).size = "big";
    else
        pos(midID).size = "small";
    end
end

imPos = double(imPos)/255;

R = imPos(:,:,1);
G = imPos(:,:,2);
B = imPos(:,:,3);
intensity = R+G+B;

g = G./intensity;
BWG = g>=imageThreshold;
%figure;imshow(g);
regionBWG = regionprops(BWG, 'Centroid');
for i=1:length(regionBWG)
    for j=1:length(pos)
        if abs(regionBWG(i).Centroid(1) - pos(j).Centroid(1))<=colourThreshold && abs(regionBWG(i).Centroid(2) - pos(j).Centroid(2))<=colourThreshold
            pos(j).Colour = "Green";
        end
    end
end
r = R./intensity;
BWR = r(:,:) >=imageThreshold;
%figure;imshow(BWR);
regionBWR = regionprops(BWR, 'Centroid');
for i=1:length(regionBWR)
    for j=1:length(pos)
        if abs(regionBWR(i).Centroid(1) - pos(j).Centroid(1))<=colourThreshold && abs(regionBWR(i).Centroid(2) - pos(j).Centroid(2))<=colourThreshold
            pos(j).Colour = "Red";
        end
    end
end

end

function [H] = homography(p1, p2)

if nargin < 2
    error('must pass uv1 and uv2');
end

if size(p1,2) ~= size(p2,2)
    error('must have same number of points in each set');
end
if size(p1,1) ~= size(p2,1)
    error('p1 and p2 must have same number of rows')
end

% linear estimation step
H = vgg_H_from_x_lin(p1, p2);

end

function H = vgg_H_from_x_lin(xs1,xs2)

[r,c] = size(xs1);

if (size(xs1) ~= size(xs2))
    error ('Input point sets are different sizes!')
end

if (size(xs1,1) == 2)
    xs1 = [xs1 ; ones(1,size(xs1,2))];
    xs2 = [xs2 ; ones(1,size(xs2,2))];
end

% condition points
C1 = vgg_conditioner_from_pts(xs1);
C2 = vgg_conditioner_from_pts(xs2);
xs1 = vgg_condition_2d(xs1,C1);
xs2 = vgg_condition_2d(xs2,C2);

D = [];
ooo  = zeros(1,3);
for k=1:c
    p1 = xs1(:,k);
    p2 = xs2(:,k);
    D = [ D;
        p1'*p2(3) ooo -p1'*p2(1)
        ooo p1'*p2(3) -p1'*p2(2)
        ];
end

% Extract nullspace
[u,s,v] = svd(D, 0); s = diag(s);

nullspace_dimension = sum(s < eps * s(1) * 1e3);
if nullspace_dimension > 1
    fprintf('Nullspace is a bit roomy...');
end

h = v(:,9);

H = reshape(h,3,3)';

%decondition
H = inv(C2) * H * C1;

H = H/H(3,3);
end
function pc = vgg_condition_2d(p,C)

[r,c] = size(p);
if r == 2
    pc = vgg_get_nonhomg(C * vgg_get_homg(p));
elseif r == 3
    pc = C * p;
else
    error ('rows != 2 or 3');
end
end
function T = vgg_conditioner_from_pts(Pts,isotropic)
Dim=size(Pts,1);

Pts=vgg_get_nonhomg(Pts);
Pts=Pts(1:Dim-1,:);

m=mean(Pts,2);
s=std(Pts');
s=s+(s==0);

if nargin==1
    T=[ diag(sqrt(2)./s) -diag(sqrt(2)./s)*m];
else % isotropic; added by TW
    T=[ diag(sqrt(2)./(ones(1,Dim-1)*mean(s))) -diag(sqrt(2)./s)*m];
end
T(Dim,:)=0;
T(Dim,Dim)=1;
end
function x = vgg_get_nonhomg(x)

if isempty(x)
    x = [];
    return;
end

d = size(x,1) - 1;
x = x(1:d,:)./(ones(d,1)*x(end,:));

return
end