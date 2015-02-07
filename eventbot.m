function [value,isterminal,direction] = eventbot(t,y,par)

value = y(5) + par(11);
isterminal = 1;
direction = -1;