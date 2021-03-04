%% Solution for EGB339 prac for 2020




function [init_xy, dest_xy] = EGB339_prac_exam_group_67(dobot_sim, init_positions, dest_positions, use_vision)
    
    if use_vision == true
        target = [0 0 -40 40 0];
        dobot_sim.setJointPositions(target);
        pause(10);

        img = dobot_sim.getImage();
        img = imcrop(img,[1 195 640 408]);

        img1 = double(img)/255;
        R = img1(:,:,1);
        G = img1(:,:,2);
        B = img1(:,:,3);
        intensity = R+G+B;

        r = R./intensity;
        g = G./intensity;
        b = B./intensity;

        %Blue Marker
        b_filtered = imbinarize(b);
        b_filtered = imfill(b_filtered,'holes');
        b_s = regionprops('table',b_filtered,'Centroid');
        b_cen = b_s.Centroid;

        %Blue Marker order arrangement 
        left_coords = [b_cen(1,:);b_cen(2,:);b_cen(3,:)];
        [x, index] = sort(left_coords(:,2));
        left_coords = [left_coords(index(1),:);left_coords(index(2),:);left_coords(index(3),:)];

        mid_coords = [b_cen(4,:);b_cen(5,:);b_cen(6,:)];
        [x, index2] = sort(mid_coords(:,2));
        mid_coords = [mid_coords(index2(1),:);mid_coords(index2(2),:);mid_coords(index2(3),:)];

        right_coords = [b_cen(7,:);b_cen(8,:);b_cen(9,:)];
        [x, index3] = sort(right_coords(:,2));
        right_coords = [right_coords(index3(1),:);right_coords(index3(2),:);right_coords(index3(3),:)];

        b_cen = [left_coords;mid_coords;right_coords];

        %Filtering Colour Green
        g_filtered = imbinarize(g);
        g_filtered = imfill(g_filtered,'holes');
        g_s = regionprops('table',g_filtered,'Centroid','Area','Perimeter');
        g_cen = g_s.Centroid;
        g_area = g_s.Area;
        g_per = g_s.Perimeter;
        first_initial = g_cen(1,:);
        third_initial = g_cen(4,:);


        %green - Shape
        g_square = [];
        g_triangle = [];
        g_circle = [];
        n = length(g_cen);

        %calculation
        for i = 1:n
             %calculate
               a = g_area(i);
               per = g_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.9 <= p && p~=Inf
                    g_circle = [g_circle;g_cen(i,:),g_area(i)];
               elseif (0.7 <= p)&&(p < 0.9)
                    g_square = [g_square;g_cen(i,:),g_area(i)];
               elseif (0.52 <= p)&&(p <= 0.69)
                    g_triangle = [g_triangle;g_cen(i,:),g_area(i)];
               end
        end


        %green - Size
        %square
        if g_square(1,3) > g_square(2,3)
            g_big_square = [g_square(1,1),g_square(1,2)];
            g_small_square = [g_square(2,1),g_square(2,2)];
        else
            g_big_square = [g_square(2,1),g_square(2,2)];
            g_small_square = [g_square(1,1),g_square(1,2)];
        end

        %circle
        if g_circle(1,3) > g_circle(2,3)
            g_big_circle = [g_circle(1,1),g_circle(1,2)];
            g_small_circle = [g_circle(2,1),g_circle(2,2)];
        else
            g_big_circle = [g_circle(2,1),g_circle(2,2)];
            g_small_circle = [g_circle(1,1),g_circle(1,2)];
        end

        %triangle
        if g_triangle(1,3) > g_triangle(2,3)
            g_big_triangle = [g_triangle(1,1),g_triangle(1,2)];
            g_small_triangle = [g_triangle(2,1),g_triangle(2,2)];
        else
            g_big_triangle = [g_triangle(2,1),g_triangle(2,2)];
            g_small_triangle = [g_triangle(1,1),g_triangle(1,2)];
        end


        %Filtering Colour Red
        r_filtered = imbinarize(r);
        r_filtered = imfill(r_filtered,'holes');
        r_s = regionprops('table',r_filtered,'Centroid','Area','Perimeter');
        r_area = r_s.Area;
        r_per = r_s.Perimeter;
        r_cen = r_s.Centroid;
        second_initial = [r_cen(4,1),r_cen(4,2)];

        %red - Shape
        r_square = [];
        r_triangle = [];
        r_circle = [];
        k = length(r_cen);


        %calculation
        for i = 1:k
             %calculate
               a = r_area(i);
               per = r_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.9 <= p && p~=Inf
                    r_circle = [r_circle;r_cen(i,:), r_area(i)];
               elseif (0.7 <= p)&&(p <= 0.9)
                    r_square = [r_square;r_cen(i,:), r_area(i)];
               elseif (0.52 <= p)&&(p <= 0.69)
                    r_triangle = [r_triangle;r_cen(i,:),r_area(i)];
               end
        end

        %red - Size
        %square
        if r_square(1,3) > r_square(2,3)
            r_big_square = [r_square(1,1),r_square(1,2)];
            r_small_square = [r_square(2,1),r_square(2,2)];
        else
            r_big_square = [r_square(2,1),r_square(2,2)];
            r_small_square = [r_square(1,1),r_square(1,2)];
        end

        %circle
        if r_circle(1,3) > r_circle(2,3)
            r_big_circle = [r_circle(1,1),r_circle(1,2)];
            r_small_circle = [r_circle(2,1),r_circle(2,2)];
        else
            r_big_circle = [r_circle(2,1),r_circle(2,2)];
            r_small_circle = [r_circle(1,1),r_circle(1,2)];
        end

        %triangle
        if r_triangle(1,3) > r_triangle(2,3)
            r_big_triangle = [r_triangle(1,1),r_triangle(1,2)];
            r_small_triangle = [r_triangle(2,1),r_triangle(2,2)];
        else
            r_big_triangle = [r_triangle(2,1),r_triangle(2,2)];
            r_small_triangle = [r_triangle(1,1),r_triangle(1,2)];
        end

        % analysis image - inital image and dest image
        %---------------------------------------------------------------------------------------------------
        %inital image
        %initial_img = imread(init_positions);
        initial_img = init_positions;
        %colour filter
        rgb_img = double(initial_img)/255;
        R_test = rgb_img(:,:,1);
        G_test = rgb_img(:,:,2);
        B_test = rgb_img(:,:,3);
        intensity = R_test + G_test + B_test;

        r_test = R_test./intensity;
        g_test = G_test./intensity;

        %coord, area
        test_img = rgb2gray(initial_img);
        test_img = ~imbinarize(test_img);
        test_img_s = regionprops('table',test_img,'Centroid','Area','Perimeter','BoundingBox');
        test_img_cen = test_img_s.Centroid;
        test_img_area = test_img_s.Area;
        test_img_per = test_img_s.Perimeter;
        test_img_bb = test_img_s.BoundingBox;

        %identify shape: Circle - 1, Square - 2, Triangle - 3
        shape_test = [];
        for i = 1:length(test_img_cen)
             %calculate
               a = test_img_area(i);
               per = test_img_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.89 <= p && p~=Inf
                    shape_test = [shape_test;test_img_cen(i,:),1;];
               elseif (0.8 <= p)&&(p <= 0.84)
                    shape_test= [shape_test;test_img_cen(i,:),2;];
               elseif (0.52 <= p)&&(p <= 0.68)
                    shape_test = [shape_test;test_img_cen(i,:),3;];
               end
        end

        %use the height from bounding box
        %use the height to find area in same shape - square
        size_test = [];
        area_test = [test_img_bb(1,4)^2,test_img_bb(2,4)^2,test_img_bb(3,4)^2];

        %find average
        average = sum(area_test)/3;

        %identify the size of the shape - big = 1, small = 0
        for i=1:length(area_test)
            if area_test(i) > average
                size_test = [size_test; 1];
            else
                size_test = [size_test; 0];
            end
        end

        %Colour identify
        %Green
        g_test = imclose(g_test,ones(15,15));
        g_test = imbinarize(g_test);
        g_test_s = regionprops('table',g_test,'Centroid','Area','Perimeter');
        g_test_cen = g_test_s.Centroid;

        %green - 1
        g_img = [];
        for i = 1:size(g_test_cen,1)
           g_img = [g_img; g_test_cen(i,:),1]; 
        end

        %Red
        r_test = imclose(r_test,ones(15,15));
        r_test = imbinarize(r_test);
        r_test_s = regionprops('table',r_test,'Centroid');
        r_test_cen = r_test_s.Centroid;

        % red - 0
        r_img = [];
        for i = 1:size(r_test_cen,1)
           r_img = [r_img; r_test_cen(i,:),0]; 
        end

        %put all coord in one array
        colour_coord = [g_img;r_img];
        %sort coord from left to right
        colour_coord = sortrows(colour_coord);
        %only save colour in array
        colour = [colour_coord(1,3);colour_coord(2,3);colour_coord(3,3)];

        %final result
        %[colour, size, shape]
        %green = 1  red = 0
        %big = 1    small = 0
        %circle = 1 sqaure = 2 triangle =3
        result = [];
        for i = 1:size(test_img_cen,1)
            result = [result;colour(i),size_test(i),shape_test(i,3)];
        end

        initial_result = [];
        for i = 1:size(result,1)
            if result(i,1) == 1 %colour - green
                if result(i,2) == 1 %size - big
                    if result(i,3) == 1     %shape - circle
                        initial_result = [initial_result; g_big_circle];    % g_big_cirlce   111
                    elseif result(i,3) == 2 %shape - square
                        initial_result = [initial_result; g_big_square];    % g_big_square   112     
                    else %shape - triangle
                        initial_result = [initial_result; g_big_triangle];  % g_big_triangle 113   
                    end
                else %size - small
                    if result(i,3) == 1     %shape - circle
                        initial_result = [initial_result; g_small_circle];  % g_small_circle   101
                    elseif result(i,3) == 2 %shape - square
                        initial_result = [initial_result; g_small_square];  % g_small_sqaure   102 
                    else %shape - triangle
                        initial_result = [initial_result; g_small_triangle];% g_small_triangle 103
                    end
                end
            else %colour - red
                if result(i,2) == 1 %size - big
                    if result(i,3) == 1     %shape - circle
                        initial_result = [initial_result; r_big_circle];    % r_big_cirlce    011
                    elseif result(i,3) == 2 %shape - square
                        initial_result = [initial_result; r_big_square];    % r_big_square    012
                    else %shape - triangle
                        initial_result = [initial_result; r_big_triangle];  % r_big_triangle  013
                    end
                else %size - small
                    if result(i,3) == 1     %shape - circle
                        initial_result = [initial_result; r_small_circle];  % r_small_circle   001
                    elseif result(i,3) == 2 %shape - square
                        initial_result = [initial_result; r_small_square];  % r_small_sqaure   002
                    else %shape - triangle
                        initial_result = [initial_result; r_small_triangle];% r_small_triangle 003
                    end
                end
            end
        end
        %------------------------------------------------------------------------------------------------------------
        %Dest image
        %dest_img = imread(dest_positions);
        dest_img = dest_positions;
        %colour filter
        rgb_img = double(dest_img)/255;
        R_test = rgb_img(:,:,1);
        G_test = rgb_img(:,:,2);
        B_test = rgb_img(:,:,3);
        intensity = R_test + G_test + B_test;

        r_test = R_test./intensity;
        g_test = G_test./intensity;

        %coord, area
        test_img = rgb2gray(dest_img);
        test_img = ~imbinarize(test_img);
        test_img_s = regionprops('table',test_img,'Centroid','Area','Perimeter','BoundingBox');
        test_img_cen = test_img_s.Centroid;
        test_img_area = test_img_s.Area;
        test_img_per = test_img_s.Perimeter;
        test_img_bb = test_img_s.BoundingBox;

        %identify shape: Circle - 1, Square - 2, Triangle - 3
        shape_test = [];
        for i = 1:length(test_img_cen)
             %calculate
               a = test_img_area(i);
               per = test_img_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.89 <= p && p~=Inf
                    shape_test = [shape_test;test_img_cen(i,:),1;];
               elseif (0.8 <= p)&&(p <= 0.84)
                    shape_test= [shape_test;test_img_cen(i,:),2;];
               elseif (0.52 <= p)&&(p <= 0.68)
                    shape_test = [shape_test;test_img_cen(i,:),3;];
               end
        end

        %use the height from bounding box
        %use the height to find area in same shape - square
        size_test = [];
        area_test = [test_img_bb(1,4)^2,test_img_bb(2,4)^2,test_img_bb(3,4)^2];

        %find average
        average = sum(area_test)/3;

        %identify the size of the shape - big = 1, small = 0
        for i=1:length(area_test)
            if area_test(i) > average
                size_test = [size_test; 1];
            else
                size_test = [size_test; 0];
            end
        end

        %Colour identify
        %Green
        g_test = imclose(g_test,ones(15,15));
        g_test = imbinarize(g_test);
        g_test_s = regionprops('table',g_test,'Centroid','Area','Perimeter');
        g_test_cen = g_test_s.Centroid;

        %green - 1
        g_img = [];
        for i = 1:size(g_test_cen,1)
           g_img = [g_img; g_test_cen(i,:),1]; 
        end

        %Red
        r_test = imclose(r_test,ones(15,15));
        r_test = imbinarize(r_test);
        r_test_s = regionprops('table',r_test,'Centroid');
        r_test_cen = r_test_s.Centroid;


        % red - 0
        r_img = [];
        for i = 1:size(r_test_cen,1)
           r_img = [r_img; r_test_cen(i,:),0]; 
        end

        %put all coord in one array
        colour_coord = [g_img;r_img];
        %sort coord from left to right
        colour_coord = sortrows(colour_coord);
        %only save colour in array
        colour = [colour_coord(1,3);colour_coord(2,3);colour_coord(3,3)];

        %final result
        %[colour, size, shape]
        %green = 1  red = 0
        %big = 1    small = 0
        %circle = 1 sqaure = 2 triangle =3
        result = [];
        for i = 1:size(test_img_cen,1)
            result = [result;colour(i),size_test(i),shape_test(i,3)];
        end

        dest_result = [];
        for i = 1:size(result,1)
            if result(i,1) == 1 %colour - green
                if result(i,2) == 1 %size - big
                    if result(i,3) == 1     %shape - circle
                        dest_result = [dest_result; g_big_circle];    % g_big_cirlce   111
                    elseif result(i,3) == 2 %shape - square
                        dest_result = [dest_result; g_big_square];    % g_big_square   112     
                    else %shape - triangle
                        dest_result = [dest_result; g_big_triangle];  % g_big_triangle 113   
                    end
                else %size - small
                    if result(i,3) == 1     %shape - circle
                        dest_result = [dest_result; g_small_circle];  % g_small_circle   101
                    elseif result(i,3) == 2 %shape - square
                        dest_result = [dest_result; g_small_square];  % g_small_sqaure   102 
                    else %shape - triangle
                        dest_result = [dest_result; g_small_triangle];% g_small_triangle 103
                    end
                end
            else %colour - red
                if result(i,2) == 1 %size - big
                    if result(i,3) == 1     %shape - circle
                        dest_result = [dest_result; r_big_circle];    % r_big_cirlce    011
                    elseif result(i,3) == 2 %shape - square
                        dest_result = [dest_result; r_big_square];    % r_big_square    012
                    else %shape - triangle
                        dest_result = [dest_result; r_big_triangle];  % r_big_triangle  013
                    end
                else %size - small
                    if result(i,3) == 1     %shape - circle
                        dest_result = [dest_result; r_small_circle];  % r_small_circle   001
                    elseif result(i,3) == 2 %shape - square
                        dest_result = [dest_result; r_small_square];  % r_small_sqaure   002
                    else %shape - triangle
                        dest_result = [dest_result; r_small_triangle];% r_small_triangle 003
                    end
                end
            end
        end
        
        %-------------------------------------------------------------------------------------
        %pixel coord to real world coord
        %homography
        P = b_cen';
        Q =[345,560    ;    182.5,560    ; 20, 560 ;
            345,  290  ;  182.5,  290    ; 20, 290  ;
            345,20  ;  182.5, 20      ; 20,20]';
        H = simple_homography(P,Q);

        %inital destination
        initial = [];
        for i = 1:size(initial_result,1)
            img_coord = [initial_result(i,:),1]';
            real_coord = H*img_coord;
            real_coord = [real_coord(1)/real_coord(3),real_coord(2)/real_coord(3)];
            initial = [initial;real_coord];
        end
        
        %destination destination
        destination = [];
        for i = 1:size(dest_result,1)
            img_coord = [dest_result(i,:),1]';
            real_coord = H*img_coord;
            real_coord = [real_coord(1)/real_coord(3),real_coord(2)/real_coord(3)];
            destination = [destination;real_coord];
        end
        
        %take away the distance of the robot from init_coord and dest_coord
        %to operate the robot arm
        %initial coordinates
        init_coord = [];
        for i = 1:size(initial,1)
           init_coord(i,:) = [initial(i,1)-80,initial(i,2)-290]; 
        end
        
        %destination coordinates
        dest_coord = [];
        for i = 1:size(destination,1)
           dest_coord(i,:) = [destination(i,1)-80,destination(i,2)-290]; 
        end
        
        %------------------------------------------------------------------------------------------
        %Calculate angle of the joint
        l1 = 138;     %Link 1 (mm)
        l2 = 135;     %Link 2 (mm)
        l3 = 147;     %Link 3 (mm)
        l4 = 60;      %Link 4 (mm)
        l5 = 75;      %Link 5 (mm)
        z = 55;       %Cylinder height (mm)
        
        %initial target
        init_target = [];
        for i = 1:size(init_coord,1)
           x = init_coord(i,1);
           y = init_coord(i,2);
           %First angle
           theta1 = atan2d(y,x);
           
           %Third angle
           lb = sqrt(((sqrt(x^2+y^2)-l4)^2)+(z + l5 - l1)^2);
           value = (l2^2 + l3^2 - lb^2)/(2*l2*l3);
           theta3 = atan2d(sqrt(1-(value^2)),value);
           theta3 = 90-theta3;
           
           %Second angle
           alpha = (lb^2 + l2^2 - l3^2)/(2*(lb)*(l2));
           alpha = atan2d(sqrt(1-(alpha^2)),alpha);
           beta = atan2d((z + l5 - l1),(sqrt((x^2)+(y^2))-l4));
           theta2 = 180-90-alpha-beta;
           
           %Fourth angle
           theta4 = -theta2-theta3;
           
           %store in array
           init_target = [init_target; theta1 0 25 theta4 0; theta1 theta2 theta3 theta4 0;];
        end
        
        %destination target
        dest_target = [];
        for i = 1:size(dest_coord,1)
           x = dest_coord(i,1);
           y = dest_coord(i,2);
           %First angle
           theta1 = atan2d(y,x);
           
           %Third angle
           lb = sqrt(((sqrt(x^2+y^2)-l4)^2)+(z + l5 - l1)^2);
           value = (l2^2 + l3^2 - lb^2)/(2*l2*l3);
           theta3 = atan2d(sqrt(1-(value^2)),value);
           theta3 = 90-theta3;
           
           %Second angle
           alpha = (lb^2 + l2^2 - l3^2)/(2*(lb)*(l2));
           alpha = atan2d(sqrt(1-(alpha^2)),alpha);
           beta = atan2d((z + l5 - l1),(sqrt((x^2)+(y^2))-l4));
           theta2 = 180-90-alpha-beta;
           
           %Fourth angle
           theta4 = -theta2-theta3;
           
           %store in array
           dest_target = [dest_target; theta1 0 25 theta4 0; theta1 theta2 theta3 theta4 0;];
        end
        final_target = dest_target(end,:);
        
        %----------------------------------------------------------------------------
        dobot_sim.setCylinderPosition(initial)
        %move all cyclinder
        n = 1:2:size(init_target,1);
        for i = n
            %first step - robot arm go to inital coord with head up
            first = init_target(i,:);
            dobot_sim.setJointPositions(first);
            pause(5);
            
            %second step - robot arm head down touch the cyclinder
            second = init_target(i+1,:);
            dobot_sim.setJointPositions(second);
            pause(5);
            
            %third step - robot arm turn on suction
            dobot_sim.setSuctionCup(true);
            pause(5);
            
            %forth step - robot arm head up to dest coord
            forth = dest_target(i,:);
            dobot_sim.setJointPositions(forth);
            pause(5);
            
            %fifth step - robot arm head down to place cyclinder
            fifth = dest_target(i+1,:);
            dobot_sim.setJointPositions(fifth);
            pause(5);
            
            %sixth step - robot arm turn off suction
            dobot_sim.setSuctionCup(false);
            pause(5);
        end
        
        %robot arm return
        final_headup = [final_target(1) 0 final_target(3) final_target(4) 0];
        dobot_sim.setJointPositions(final_headup);
        pause(5);
        
        final_return = [0 0 0 0 0];
        dobot_sim.setJointPositions(final_return);
        pause(3);

        % you will need to calculate the xy positions and return them at the end of this section of code
        % shapes_xy needs to be 
        % init_shape_xy and dest_xy need to be of size 3 x 2 and an example is shown below
        init_xy = initial;
        dest_xy = destination;
        
    else

        %% Execute Pick and Place

        %Robot Vision
        target = [0 0 -40 40 0];
        dobot_sim.setJointPositions(target);
        pause(10);

        img = dobot_sim.getImage();
        img = imcrop(img,[1 195 640 408]);

        img1 = double(img)/255;
        R = img1(:,:,1);
        G = img1(:,:,2);
        B = img1(:,:,3);
        intensity = R+G+B;

        r = R./intensity;
        g = G./intensity;
        b = B./intensity;

        %Blue Marker
        b_filtered = imbinarize(b);
        b_filtered = imfill(b_filtered,'holes');
        b_s = regionprops('table',b_filtered,'Centroid');
        b_cen = b_s.Centroid;

        %Blue Marker order arrangement 
        left_coords = [b_cen(1,:);b_cen(2,:);b_cen(3,:)];
        [x, index] = sort(left_coords(:,2));
        left_coords = [left_coords(index(1),:);left_coords(index(2),:);left_coords(index(3),:)];

        mid_coords = [b_cen(4,:);b_cen(5,:);b_cen(6,:)];
        [x, index2] = sort(mid_coords(:,2));
        mid_coords = [mid_coords(index2(1),:);mid_coords(index2(2),:);mid_coords(index2(3),:)];

        right_coords = [b_cen(7,:);b_cen(8,:);b_cen(9,:)];
        [x, index3] = sort(right_coords(:,2));
        right_coords = [right_coords(index3(1),:);right_coords(index3(2),:);right_coords(index3(3),:)];

        b_cen = [left_coords;mid_coords;right_coords];

        %Filtering Colour Green
        g_filtered = imbinarize(g);
        g_filtered = imfill(g_filtered,'holes');
        g_s = regionprops('table',g_filtered,'Centroid','Area','Perimeter');
        g_cen = g_s.Centroid;
        g_area = g_s.Area;
        g_per = g_s.Perimeter;
        first_initial = g_cen(1,:);
        third_initial = g_cen(4,:);


        %green - Shape
        g_square = [];
        g_triangle = [];
        g_circle = [];
        n = length(g_cen);

        %calculation
        for i = 1:n
             %calculate
               a = g_area(i);
               per = g_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.9 <= p && p~=Inf
                    g_circle = [g_circle;g_cen(i,:),g_area(i)];
               elseif (0.7 <= p)&&(p < 0.9)
                    g_square = [g_square;g_cen(i,:),g_area(i)];
               elseif (0.52 <= p)&&(p <= 0.69)
                    g_triangle = [g_triangle;g_cen(i,:),g_area(i)];
               end
        end


        %green - Size
        %square
        if g_square(1,3) > g_square(2,3)
            g_big_square = [g_square(1,1),g_square(1,2)];
            g_small_square = [g_square(2,1),g_square(2,2)];
        else
            g_big_square = [g_square(2,1),g_square(2,2)];
            g_small_square = [g_square(1,1),g_square(1,2)];
        end

        %circle
        if g_circle(1,3) > g_circle(2,3)
            g_big_circle = [g_circle(1,1),g_circle(1,2)];
            g_small_circle = [g_circle(2,1),g_circle(2,2)];
        else
            g_big_circle = [g_circle(2,1),g_circle(2,2)];
            g_small_circle = [g_circle(1,1),g_circle(1,2)];
        end

        %triangle
        if g_triangle(1,3) > g_triangle(2,3)
            g_big_triangle = [g_triangle(1,1),g_triangle(1,2)];
            g_small_triangle = [g_triangle(2,1),g_triangle(2,2)];
        else
            g_big_triangle = [g_triangle(2,1),g_triangle(2,2)];
            g_small_triangle = [g_triangle(1,1),g_triangle(1,2)];
        end


        %Filtering Colour Red
        r_filtered = imbinarize(r);
        r_filtered = imfill(r_filtered,'holes');
        r_s = regionprops('table',r_filtered,'Centroid','Area','Perimeter');
        r_area = r_s.Area;
        r_per = r_s.Perimeter;
        r_cen = r_s.Centroid;
        second_initial = [r_cen(4,1),r_cen(4,2)];

        %red - Shape
        r_square = [];
        r_triangle = [];
        r_circle = [];
        k = length(r_cen);


        %calculation
        for i = 1:k
             %calculate
               a = r_area(i);
               per = r_per(i);
               p = (4*pi*a)/(per^2);    
               if 0.9 <= p && p~=Inf
                    r_circle = [r_circle;r_cen(i,:), r_area(i)];
               elseif (0.7 <= p)&&(p <= 0.9)
                    r_square = [r_square;r_cen(i,:), r_area(i)];
               elseif (0.52 <= p)&&(p <= 0.69)
                    r_triangle = [r_triangle;r_cen(i,:),r_area(i)];
               end
        end

        %red - Size
        %square
        if r_square(1,3) > r_square(2,3)
            r_big_square = [r_square(1,1),r_square(1,2)];
            r_small_square = [r_square(2,1),r_square(2,2)];
        else
            r_big_square = [r_square(2,1),r_square(2,2)];
            r_small_square = [r_square(1,1),r_square(1,2)];
        end

        %circle
        if r_circle(1,3) > r_circle(2,3)
            r_big_circle = [r_circle(1,1),r_circle(1,2)];
            r_small_circle = [r_circle(2,1),r_circle(2,2)];
        else
            r_big_circle = [r_circle(2,1),r_circle(2,2)];
            r_small_circle = [r_circle(1,1),r_circle(1,2)];
        end

        %triangle
        if r_triangle(1,3) > r_triangle(2,3)
            r_big_triangle = [r_triangle(1,1),r_triangle(1,2)];
            r_small_triangle = [r_triangle(2,1),r_triangle(2,2)];
        else
            r_big_triangle = [r_triangle(2,1),r_triangle(2,2)];
            r_small_triangle = [r_triangle(1,1),r_triangle(1,2)];
        end
        
        %-------------------------------------------------------------------------------------
        %pixel coord to real world coord
        %homography
        P = b_cen';
        Q =[345,560    ;    182.5,560    ; 20, 560 ;
            345,  290  ;  182.5,  290    ; 20, 290  ;
            345,20  ;  182.5, 20      ; 20,20]';
        H = simple_homography(P,Q);

        %inital destination
        initial = [];
        for i = 1:size(init_positions,1)
            img_coord = [init_positions(i,:),1]';
            real_coord = H*img_coord;
            real_coord = [real_coord(1)/real_coord(3),real_coord(2)/real_coord(3)];
            initial = [initial;real_coord];
        end
        
        %destination destination
        destination = [];
        for i = 1:size(dest_positions,1)
            img_coord = [dest_positions(i,:),1]';
            real_coord = H*img_coord;
            real_coord = [real_coord(1)/real_coord(3),real_coord(2)/real_coord(3)];
            destination = [destination;real_coord];
        end
        
        %take away the distance of the robot from init_coord and dest_coord
        %to operate the robot arm
        %initial coordinates
        init_coord = [];
        for i = 1:size(initial,1)
           init_coord(i,:) = [initial(i,1)-80,initial(i,2)-290]; 
        end
        
        %destination coordinates
        dest_coord = [];
        for i = 1:size(destination,1)
           dest_coord(i,:) = [destination(i,1)-80,destination(i,2)-290]; 
        end
        
        %------------------------------------------------------------------------------------------
        %Calculate angle of the joint
        l1 = 138;     %Link 1 (mm)
        l2 = 135;     %Link 2 (mm)
        l3 = 147;     %Link 3 (mm)
        l4 = 60;      %Link 4 (mm)
        l5 = 75;      %Link 5 (mm)
        z = 55;       %Cylinder height (mm)
        
        %initial target
        init_target = [];
        for i = 1:size(init_coord,1)
           x = init_coord(i,1);
           y = init_coord(i,2);
           %First angle
           theta1 = atan2d(y,x);
           
           %Third angle
           lb = sqrt(((sqrt(x^2+y^2)-l4)^2)+(z + l5 - l1)^2);
           value = (l2^2 + l3^2 - lb^2)/(2*l2*l3);
           theta3 = atan2d(sqrt(1-(value^2)),value);
           theta3 = 90-theta3;
           
           %Second angle
           alpha = (lb^2 + l2^2 - l3^2)/(2*(lb)*(l2));
           alpha = atan2d(sqrt(1-(alpha^2)),alpha);
           beta = atan2d((z + l5 - l1),(sqrt((x^2)+(y^2))-l4));
           theta2 = 180-90-alpha-beta;
           
           %Fourth angle
           theta4 = -theta2-theta3;
           
           %store in array
           init_target = [init_target; theta1 0 theta3 theta4 0; theta1 theta2 theta3 theta4 0;];
        end
        
        %destination target
        dest_target = [];
        
        for i = 1:size(dest_coord,1)
           x = dest_coord(i,1);
           y = dest_coord(i,2);
           %First angle
           theta1 = atan2d(y,x);
           
           %Third angle
           lb = sqrt(((sqrt(x^2+y^2)-l4)^2)+(z + l5 - l1)^2);
           value = (l2^2 + l3^2 - lb^2)/(2*l2*l3);
           theta3 = atan2d(sqrt(1-(value^2)),value);
           theta3 = 90-theta3;
           
           %Second angle
           alpha = (lb^2 + l2^2 - l3^2)/(2*(lb)*(l2));
           alpha = atan2d(sqrt(1-(alpha^2)),alpha);
           beta = atan2d((z + l5 - l1),(sqrt((x^2)+(y^2))-l4));
           theta2 = 180-90-alpha-beta;
           
           %Fourth angle
           theta4 = -theta2-theta3;
           
           %store in array
           dest_target = [dest_target; theta1 0 theta3 theta4 0; theta1 theta2 theta3 theta4 0;];
        end
        
        %final_target
        final_target = dest_target(end,:);
        
        %---------------------------------------------------------------------
        %move all cyclinder
        dobot_sim.setCylinderPosition(initial);
        n = 1:2:size(init_target,1);
        for i = n
            %first step - robot arm go to inital coord with head up
            first = init_target(i,:);
            dobot_sim.setJointPositions(first);
            pause(5);
            
            %second step - robot arm head down touch the cyclinder
            second = init_target(i+1,:);
            dobot_sim.setJointPositions(second);
            pause(5);
            
            %third step - robot arm turn on suction
            dobot_sim.setSuctionCup(true);
            pause(5);
            %forth step - robot arm head up to dest coord
            forth = dest_target(i,:);
            dobot_sim.setJointPositions(forth);
            pause(5);
            
            %fifth step - robot arm head down to place cyclinder
            fifth = dest_target(i+1,:);
            dobot_sim.setJointPositions(fifth);
            pause(5);
            
            %sixth step - robot arm turn off suction
            dobot_sim.setSuctionCup(false);
            pause(5);
        end
        
        %robot arm return
        final_headup = [final_target(1) 0 final_target(3) final_target(4) 0];
        dobot_sim.setJointPositions(final_headup);
        pause(5);
        
        final_return = [0 0 0 0 0];
        dobot_sim.setJointPositions(final_return);
        pause(3);
        

        % you will need to calculate the xy positions and return them at the end of this section of code
        % shapes_xy needs to be 
        % init_shape_xy and dest_xy need to be of size 3 x 2 and an example is shown below
        init_xy = initial;
        dest_xy = destination;

        
    end
    

end