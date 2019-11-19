function euler = quatern2euler(q)
    % from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    % Heading Reference System" by Wang et al, IEEE.
    
%     R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
%     R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
%     R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
%     R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
%     R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;    

%     R(1,1,:) = cos(q(:,1)) + q(:,2).*q(:,2).*(1-cos(q(:,1)));
%     R(2,1,:) = q(:,2).*q(:,3).*(1-cos(q(:,1))) - q(:,4).*sin(q(:,1));
%     R(3,1,:) = q(:,2).*q(:,4).*(1-cos(q(:,1))) + q(:,3).*sin(q(:,1));
%     R(3,2,:) = q(:,3).*q(:,4).*(1-cos(q(:,1))) - q(:,2).*sin(q(:,1));
%     R(3,3,:) = cos(q(:,1)) + q(:,4).*q(:,4).*(1-cos(q(:,1)));
% 
%     phi = atan2(R(3,2,:), R(3,3,:) );
%     theta = atan2(R(3,1,:).*(-1), sqrt(R(3,2,:).*R(3,2,:) + R(3,3,:).*R(3,3,:)) );    
%     psi = atan2(R(2,1,:), R(1,1,:) );

%     phi = atan2((q(1).*q(2) + q(3).*q(4)).*2, q(1).*q(1) - q(2).*q(2) - q(3).*q(3) + q(4).*q(4));
%     theta = asin((q(1).*q(3) - q(4).*q(2)).*2);
%     psi = atan2((q(1).*q(4) + q(2).*q(3)).*2, q(1).*q(1) + q(2).*q(2) - q(3).*q(3) - q(4).*q(4));

    q00 = q(1).*q(1);
    q11 = q(2).*q(2);
    q22 = q(3).*q(3);
    q33 = q(4).*q(4);
    
    r11 = q00 + q11 - q22 - q33;
    r21 = 2 .* (q(2).*q(3) + q(1)*q(4));
    r31 = 2 .* (q(2).*q(4) - q(1).*q(3));
    r32 = 2 .* (q(3).*q(4) + q(1).*q(2));
    r33 = q00 - q11 - q22 + q33;
    
    tmp = abs(r31);
    if tmp > 0.999999
        r12 = 2.*(q(2).*q(3) - q(1).*q(4)); 
        r13 = 2.*(q(2).*q(4) + q(1).*q(3));
        
        Pitch = 0;
        Yaw = ((pi/2).*r31/tmp).*(-1);
        Roll = atan2(-r12, -r31.*r13);
    else
        Pitch = atan2(r32, r33);
        Yaw = asin(-r31);
        Roll = atan2(r21, r11);
    end

    euler = [Roll Pitch Yaw]; 
%     euler = [phi(1,:)' theta(1,:)' psi(1,:)']; 
end

