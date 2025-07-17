function [value, isterminal, direction] = groundImpact(t,state)
    value = state(3); % extracts the height above zero from the state
    isterminal = 1; % terminate the solver when impact with the ground occurs
    direction = -1; % only terminate if solver crosses zero in the positive direction
end