function [v] = collideAndCoalesce(m1, v1, m2, v2)
%COLLIDEANDCOALESCE Calculates the velocity of two objects after a
%collision using conservation of momentum, assuming they coalesce.
%   collideAndCoalesce(m1, v1, m2, v2) returns the velocity vector of two
%   coalesced objects of mass m1 and m2 after colliding with velocity
%   vectors v1 and v2 respectively.

v = (m1*v1 + m2*v2)/(m1+m2);

end

