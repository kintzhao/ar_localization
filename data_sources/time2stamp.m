[row col] = size(time);
odom_count = 1;
img_count = 1;
for i=1:row
    if time(i,2) == 2
        img(img_count) = time(i,3);
        img_count = img_count + 1;
    else
        img(img_count) = 0;
        img_count = img_count + 1;        
    end
    if time(i,3) == 1
        odom(odom_count) = time(i,4);
        odom_count = odom_count + 1;
    else
        odom(odom_count) = 0;
        odom_count = odom_count + 1;        
    end
end
   
img = img';
odom = odom';