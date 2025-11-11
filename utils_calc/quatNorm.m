function q = quatNorm(q_original)   
% Ensures the quaternion is normalized
    norm_q = norm(q_original);
    if norm_q ~= 0
        q = q_original / norm_q;
    end
end 